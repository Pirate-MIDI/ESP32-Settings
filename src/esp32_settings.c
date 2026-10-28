#include "esp32_settings.h"
#include "esp32_settings_tlv.h"   // shared on-disk header + TLV codec

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>

#include "esp_log.h"
#include "esp_system.h"     // esp_restart
#include "esp_littlefs.h"   // joltwallet/littlefs VFS
#include "esp_rom_crc.h"    // esp_rom_crc32_le
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  // vTaskDelay

// ---------------------------------------------------------------------------
// Storage configuration
//
// The settings live as two binary blobs on a LittleFS mount of the `storage`
// partition. This library OWNS that mount — nothing else should mount `storage`
// (on the N16R8/WiFi build the manager's WebUI also wants a `storage` LittleFS
// image; resolve that ownership then — see docs/STATUS.md).
// ---------------------------------------------------------------------------
#define SETTINGS_PARTITION_LABEL   "storage"
#define SETTINGS_BASE_PATH         "/lfs"
#define GLOBAL_PATH                SETTINGS_BASE_PATH "/global.txt"
#define GLOBAL_TMP_PATH            SETTINGS_BASE_PATH "/global.tmp"
#define BANKS_PATH                 SETTINGS_BASE_PATH "/banks.txt"
#define BANKS_TMP_PATH             SETTINGS_BASE_PATH "/banks.tmp"
#define PRESETS_PATH               SETTINGS_BASE_PATH "/presets.txt"
#define PRESETS_TMP_PATH           SETTINGS_BASE_PATH "/presets.tmp"

#define DEVICE_CONFIGURED_VALUE    114 // Arbitrary value to indicate the device has been configured

// On-disk integrity header prepended to each blob. Validation is magic + version
// + exact length + CRC32 of the payload — a corrupt or truncated file (e.g. a
// power loss mid-write) is detected and the caller falls back to defaults, rather
// than the old size-only check which a half-written file of the right length passed.
// The header struct + SETTINGS_MAGIC now live in esp32_settings_tlv.h (shared with
// the tagged path). Version 1 = raw struct blob (this legacy path); version 2 = TLV.
#define SETTINGS_VERSION   1

static const char *TAG = "ESP32_SETTINGS";

void* globalSettingsPtr = NULL;
void* banksPtr = NULL;
void* presetsPtr = NULL;
uint8_t* bootFlagPtr = NULL;
size_t numBanks = 0;
size_t numPresets = 0;

uint16_t globalSettingsSize = 0;
uint16_t bankSize = 0;
uint16_t presetSize = 0;

void (*assignDefaultGlobalSettings)() = NULL;
void (*assignDefaultBankSettings)() = NULL;
void (*assignDefaultPresetSettings)() = NULL;

// Tagged (TLV) mode. When BootCheckTagged wires these up, the global/preset
// Save*/Read* helpers persist via the forward-compatible TLV codec instead of
// the raw struct blob; banks remain raw-only. See esp32_settings_tlv.h.
static bool                    tlvMode = false;
static settings_serialize_fn   tlvGlobalSerialize   = NULL;
static settings_deserialize_fn tlvGlobalDeserialize = NULL;
static settings_serialize_fn   tlvPresetSerialize   = NULL;
static settings_deserialize_fn tlvPresetDeserialize = NULL;
static void                   *tlvCtx = NULL;

// Phased boot-check state, carried between BootBeginTagged and BootFinishTagged.
static bool                    tlvBeginOk      = false;   // Begin ran and pointers were valid
static bool                    tlvMounted      = false;   // storage mounted in Begin
static settings_tlv_status_t   tlvGlobalStatus = SETTINGS_TLV_OK;  // global-load result from Begin

// Optional "about to factory reset" notification (see esp32Settings_SetFactoryResetNotify).
static void (*factoryResetNotify)(void) = NULL;

// Grouped preset files (esp32Settings_UsePresetFiles). PRESETS_PATH is unused in that mode.
// File gPP-NNN.tlv (PP = presets per file) holds presets [NNN*PP, NNN*PP + PP), one container
// record each. PP is in the name so files from a different grouping never get misread:
//   [PRESET_ITEM_TAG][len] [u16 preset index] [that preset's own records ...]
#define PRESET_DIR                 SETTINGS_BASE_PATH "/presets"
#define PRESET_ITEM_TAG            0x0001
#define PRESETS_PER_FILE_MAX       32      // sGroupSeen is a 32-bit mask
static bool                         presetFiles      = false;
static uint16_t                     presetItemCount  = 0;   // presets covered
static uint16_t                     presetsPerFile   = 1;
static uint16_t                     presetGroupCount = 0;
static settings_item_serialize_fn   itemSerialize    = NULL;
static settings_item_deserialize_fn itemDeserialize  = NULL;
static void                       (*defaultOnePreset)(uint16_t) = NULL;
// The TLV file helpers take index-free callbacks; these carry the group being worked on.
static uint16_t                     sGroup           = 0;
static uint16_t                     sOnlyIndex       = UINT16_MAX;  // ReadPreset: decode just this one
static uint32_t                     sGroupSeen       = 0;           // bit per preset found in the file
static bool                         sGroupFileMissing = false;      // a group file was absent (layout change)

static void presetGroupRange(uint16_t group, uint16_t *first, uint16_t *end)
{
	*first = group * presetsPerFile;
	*end   = *first + presetsPerFile;
	if (*end > presetItemCount)
		*end = presetItemCount;
}

static void presetGroupName(uint16_t group, char *name)
{
	sprintf(name, "g%02u-%03u.tlv", (unsigned)presetsPerFile, (unsigned)group);
}

static void presetGroupPaths(uint16_t group, char *path, char *tmp)
{
	sprintf(path, PRESET_DIR "/g%02u-%03u.tlv", (unsigned)presetsPerFile, (unsigned)group);
	sprintf(tmp,  PRESET_DIR "/g%02u-%03u.tmp", (unsigned)presetsPerFile, (unsigned)group);
}

static bool presetGroupSerialize(tlv_writer_t *w, void *ctx)
{
	uint16_t first, end;
	presetGroupRange(sGroup, &first, &end);
	for (uint16_t i = first; i < end; i++) {
		// Dry run to size the container, then the real write; the serialiser must be
		// deterministic (it only reads RAM), which the length check below enforces.
		tlv_writer_t c;
		esp32SettingsTlv_BeginCount(&c);
		esp32SettingsTlv_WriteRaw(&c, &i, sizeof i);
		if (!itemSerialize(&c, i, ctx) || c.len > UINT16_MAX) {
			ESP_LOGE(TAG, "Preset %u: serialise failed or > 64 KB.", (unsigned)i);
			return false;
		}
		esp32SettingsTlv_WriteRecordHeader(w, PRESET_ITEM_TAG, (uint16_t)c.len);
		uint32_t start = w->len;
		esp32SettingsTlv_WriteRaw(w, &i, sizeof i);
		if (!itemSerialize(w, i, ctx))
			return false;
		if (w->ok && w->len - start != c.len) {
			ESP_LOGE(TAG, "Preset %u: serialiser wrote %u bytes, sized %u.",
			         (unsigned)i, (unsigned)(w->len - start), (unsigned)c.len);
			return false;
		}
	}
	return true;
}

static bool presetGroupDeserialize(tlv_reader_t *r, void *ctx)
{
	uint16_t first, end;
	presetGroupRange(sGroup, &first, &end);
	uint16_t tag, len;
	while (esp32SettingsTlv_NextRecord(r, &tag, &len)) {
		if (tag != PRESET_ITEM_TAG || len < sizeof(uint16_t))
			continue;   // unknown top-level record: NextRecord drains it
		uint16_t idx;
		if (esp32SettingsTlv_ReadRaw(r, &idx, sizeof idx) != sizeof idx)
			break;      // short read: FinishRead reports it
		if (idx < first || idx >= end || (sOnlyIndex != UINT16_MAX && idx != sOnlyIndex))
			continue;
		esp32SettingsTlv_EnterRecord(r);
		bool ok = itemDeserialize(r, idx, ctx);
		esp32SettingsTlv_ExitRecord(r);
		if (!ok)
			return false;
		sGroupSeen |= 1u << (idx - first);
	}
	return true;
}

static void presetGroupDefault(uint16_t group)
{
	uint16_t first, end;
	presetGroupRange(group, &first, &end);
	for (uint16_t i = first; i < end; i++)
		if (sOnlyIndex == UINT16_MAX || i == sOnlyIndex)
			defaultOnePreset(i);
}

// Delete every file in PRESET_DIR that is not one of the current layout's group files: older
// layouts (beta.2 pNNN.tlv, beta.3 gNNN.tlv, another perFile) and stray .tmp files. They hold
// flash and lengthen every lookup in the directory. Called only when a group file was missing,
// i.e. the first boot after a layout change. Entries are collected first and removed after
// closedir, because removing while iterating a littlefs directory can skip entries.
static void presetDirPrune(void)
{
	char batch[8][40];
	uint16_t removed = 0;
	for (;;) {
		DIR *dir = opendir(PRESET_DIR);
		if (dir == NULL)
			return;
		uint8_t n = 0;
		struct dirent *e;
		while (n < 8 && (e = readdir(dir)) != NULL) {
			unsigned pp, g;
			char tail[8];
			bool current = (sscanf(e->d_name, "g%2u-%3u.%7s", &pp, &g, tail) == 3) &&
			               pp == presetsPerFile && g < presetGroupCount && strcmp(tail, "tlv") == 0;
			char want[24];
			if (current) {
				presetGroupName((uint16_t)g, want);
				current = (strcmp(e->d_name, want) == 0);   // exact spelling, e.g. no "g16-0.tlv"
			}
			if (!current && e->d_name[0] != '.' && strlen(e->d_name) < sizeof batch[0])
				strcpy(batch[n++], e->d_name);
		}
		closedir(dir);
		if (n == 0)
			break;
		uint8_t gone = 0;
		for (uint8_t i = 0; i < n; i++) {
			char path[64];
			sprintf(path, PRESET_DIR "/%s", batch[i]);
			if (remove(path) == 0)
				gone++;
			else
				ESP_LOGW(TAG, "Could not remove %s", path);
		}
		removed += gone;
		if (n < 8 || gone == 0)
			break;   // directory done, or stuck on files that will not delete
	}
	if (removed)
		ESP_LOGI(TAG, "Removed %u file(s) from an older preset layout.", (unsigned)removed);
}

static bool presetGroupSave(uint16_t group)
{
	struct stat st;
	if (stat(PRESET_DIR, &st) != 0 && mkdir(PRESET_DIR, 0775) != 0) {
		ESP_LOGE(TAG, "Could not create %s", PRESET_DIR);
		return false;
	}
	char path[48], tmp[48];
	presetGroupPaths(group, path, tmp);
	sGroup = group;
	if (!esp32SettingsTlv_SaveFile(path, tmp, presetGroupSerialize, tlvCtx)) {
		ESP_LOGE(TAG, "Preset group %u save failed.", (unsigned)group);
		return false;
	}
	return true;
}

// Load one group file (all of it, or only sOnlyIndex). Presets absent from the file keep
// defaults. Returns false if the file was missing/invalid or lacked any requested preset.
static bool presetGroupRead(uint16_t group)
{
	uint16_t first, end;
	presetGroupRange(group, &first, &end);
	char path[48], tmp[48];
	presetGroupPaths(group, path, tmp);

	presetGroupDefault(group);   // defaults under any tag / preset absent from the file
	sGroup = group;
	sGroupSeen = 0;
	settings_tlv_status_t st = esp32SettingsTlv_LoadFile(path, presetGroupDeserialize, tlvCtx);
	if (st == SETTINGS_TLV_OK) {
		uint32_t want = (sOnlyIndex != UINT16_MAX) ? (1u << (sOnlyIndex - first))
		                                           : ((1u << (end - first)) - 1u);
		if ((sGroupSeen & want) == want)
			return true;
		ESP_LOGW(TAG, "Preset group %u: preset(s) missing from file — defaults.", (unsigned)group);
		return false;
	}
	// A corrupt file is detected only after its records were decoded, so re-default to drop
	// whatever was half-applied.
	presetGroupDefault(group);
	if (st == SETTINGS_TLV_MISSING) {
		ESP_LOGW(TAG, "Preset group %u: no file — defaults.", (unsigned)group);
		sGroupFileMissing = true;
	} else {
		ESP_LOGW(TAG, "Preset group %u: file invalid (status %d) — defaults.", (unsigned)group, (int)st);
	}
	return false;
}

static void esp32Settings_ListDir(const char *dirname, uint8_t levels);

// Mount the storage partition. Returns true if mounted (or already mounted).
// format_if_mount_failed lets a blank/corrupt filesystem come up formatted.
static bool esp32Settings_Mount(void)
{
	esp_vfs_littlefs_conf_t conf = {
		.base_path = SETTINGS_BASE_PATH,
		.partition_label = SETTINGS_PARTITION_LABEL,
		.format_if_mount_failed = true,
		.dont_mount = false,
	};
	esp_err_t err = esp_vfs_littlefs_register(&conf);
	if (err == ESP_ERR_INVALID_STATE) {
		// Already registered (e.g. a re-entry) — treat as mounted.
		return true;
	}
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "LittleFS mount of '%s' failed: %s",
				 SETTINGS_PARTITION_LABEL, esp_err_to_name(err));
		return false;
	}
	return true;
}

// Write header + payload to a temp file, then atomically rename over the target.
// LittleFS commits the data on fclose() and rename() is atomic, so an interrupted
// save can never leave a half-written destination file.
static bool esp32Settings_WriteBlob(const char *path, const char *tmp_path,
									 const void *data, size_t len)
{
	FILE *f = fopen(tmp_path, "wb");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open %s for write", tmp_path);
		return false;
	}

	settings_header_t hdr = {
		.magic = SETTINGS_MAGIC,
		.version = SETTINGS_VERSION,
		.reserved = 0,
		.length = (uint32_t)len,
		.crc32 = esp_rom_crc32_le(0, (const uint8_t *)data, (uint32_t)len),
	};

	bool ok = (fwrite(&hdr, 1, sizeof hdr, f) == sizeof hdr) &&
			  (fwrite(data, 1, len, f) == len);
	fclose(f); // commits the temp file to flash

	if (!ok) {
		// A short write here is almost always heap exhaustion in the caller, not a full
		// filesystem: the littlefs write needs a ~4 KB block, so if free internal heap is
		// starved (e.g. a large ArduinoJson doc still alive) the first fwrite returns 0.
		// Callers must release transient buffers before saving. See docs/SOLUTIONS.md.
		ESP_LOGE(TAG, "Short write to %s (expected %u payload bytes) — low heap?", tmp_path, (unsigned)len);
		remove(tmp_path);
		return false;
	}
	if (rename(tmp_path, path) != 0) {
		ESP_LOGE(TAG, "rename %s -> %s failed", tmp_path, path);
		remove(tmp_path);
		return false;
	}
	ESP_LOGI(TAG, "Wrote %u payload bytes to %s", (unsigned)len, path);
	return true;
}

// Read + verify a blob into `data`. Returns false (leaving as much of `data`
// possibly-partially-filled — caller reseeds on failure) if the file is missing,
// has the wrong magic/version, a mismatched length, or a bad CRC.
static bool esp32Settings_ReadBlob(const char *path, void *data, size_t len)
{
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		ESP_LOGW(TAG, "%s not present", path);
		return false;
	}

	settings_header_t hdr;
	bool ok = (fread(&hdr, 1, sizeof hdr, f) == sizeof hdr) &&
			  (hdr.magic == SETTINGS_MAGIC) &&
			  (hdr.version == SETTINGS_VERSION) &&
			  (hdr.length == len) &&
			  (fread(data, 1, len, f) == len) &&
			  (esp_rom_crc32_le(0, (const uint8_t *)data, (uint32_t)len) == hdr.crc32);
	fclose(f);

	if (!ok) {
		ESP_LOGW(TAG, "%s invalid (magic/version/size/CRC mismatch)", path);
	}
	return ok;
}

// Checks for the standard combination of a global settings file plus optional bank
// and preset files. If any used file is missing or fails validation, the device is
// reconfigured to defaults. Also initialises the global/bank/preset pointers and their
// element counts. The bootflag is a pointer into the global settings struct (its stored
// boot-state byte).
//
// `banks` and/or `presets` may be passed as NULL: a NULL element is ignored entirely —
// no size validation, existence check, load or save is performed for it. `globalSettings`
// and `bootFlag` are always required.
uint8_t esp32Settings_BootCheck(	void* globalStore, uint16_t gSize,
										void* banks, uint16_t bSize, size_t numBank,
										void* presets, uint16_t pSize, size_t numPreset,
										uint8_t* bootFlag)
{
	uint8_t bootFlagValue = 0;
	ESP_LOGI(TAG, "Bank size %d, Preset size %d", bSize, pSize);
	ESP_LOGI(TAG, "Boot check initiated.");
	if (globalStore == NULL || bootFlag == NULL) {
		ESP_LOGE(TAG, "Invalid global settings or boot flag pointer.");
		return 0;
	}

	// Assign the global/bank/preset pointers (banks/presets may be NULL = unused)
	globalSettingsPtr = globalStore;
	banksPtr = banks;
	presetsPtr = presets;
	bootFlagPtr = bootFlag;
	numBanks = numBank;
	numPresets = numPreset;
	globalSettingsSize = gSize;
	bankSize = bSize;
	presetSize = pSize;

	if (banksPtr == NULL)
		ESP_LOGI(TAG, "Banks not used (NULL) — skipping.");
	if (presetsPtr == NULL)
		ESP_LOGI(TAG, "Presets not used (NULL) — skipping.");

	// Mount the filesystem (formats a blank/corrupt partition automatically).
	ESP_LOGI(TAG, "Mounting storage...");
	if (!esp32Settings_Mount()) {
		// No usable filesystem at all — nothing we can persist to. Reconfigure
		// tries a format + rewrite; if the partition itself is absent this will
		// keep failing, which is the correct loud failure mode.
		esp32Settings_NewDeviceConfig();
	}

	// Validate + load the used files. A missing/corrupt file triggers reconfiguration
	// (which formats, writes defaults, and reboots — so it does not return). NULL
	// (unused) elements validate as OK without touching storage.
	ESP_LOGI(TAG, "Validating stored settings...");
	bool globalOk  = esp32Settings_ReadBlob(GLOBAL_PATH, globalSettingsPtr, globalSettingsSize);
	bool banksOk   = true;
	if (banksPtr != NULL)
		banksOk = globalOk && esp32Settings_ReadBlob(BANKS_PATH, banksPtr,
													 (size_t)bankSize * numBanks);
	bool presetsOk = true;
	if (presetsPtr != NULL)
		presetsOk = globalOk && esp32Settings_ReadBlob(PRESETS_PATH, presetsPtr,
													    (size_t)presetSize * numPresets);
	if (!globalOk || !banksOk || !presetsOk) {
		ESP_LOGI(TAG, "Stored settings missing/invalid — reconfiguring.");
		esp32Settings_NewDeviceConfig();
	}

	// Both files loaded into the struct pointers; the boot flag now reflects storage.
	bootFlagValue = *bootFlagPtr;
	if (*bootFlagPtr != DEVICE_CONFIGURED_VALUE && *bootFlagPtr != DEVICE_FRESH_RESET_VALUE) {
		ESP_LOGI(TAG, "Configuring new device...");
		esp32Settings_NewDeviceConfig();
	} else {
		ESP_LOGI(TAG, "Performing standard boot...");
		esp32Settings_StandardBoot();
	}
	return bootFlagValue;
}

// Tagged (TLV) boot check — forward-compatible variant of esp32Settings_BootCheck.
//
// Instead of validating a raw struct blob by exact length, it decodes a tagged
// stream via the caller's (de)serialise callbacks: adding / removing / reordering
// a field never wipes user settings (see docs/settings-migration-plan.md). Only a
// missing file, a non-TLV/wrong-version header (e.g. a legacy raw v1 blob — the
// one intended wipe at cutover), or a CRC mismatch routes to a factory reconfigure.
//
// The caller MUST have already registered the default-assignment callbacks
// (esp32Settings_AssignDefault*). Defaults are applied *before* each decode so any
// tag missing from an older file keeps its chosen default. Banks are not supported
// in tagged mode (pass presets only); `presets`/its callbacks may be NULL.
// Phase 1: mount storage + load global settings. See esp32_settings.h for the phased
// contract. After this returns, globalStore holds the stored (or defaulted) global values,
// so early-boot code (e.g. display rotation) can run before the slower Phase 2.
void esp32Settings_BootBeginTagged(
		void *globalStore, settings_serialize_fn gSerialize, settings_deserialize_fn gDeserialize,
		void *presets, settings_serialize_fn pSerialize, settings_deserialize_fn pDeserialize,
		uint8_t *bootFlag, void *ctx)
{
	ESP_LOGI(TAG, "Tagged boot check (phase 1: mount + global).");
	if (globalStore == NULL || bootFlag == NULL ||
	    gSerialize == NULL || gDeserialize == NULL) {
		ESP_LOGE(TAG, "Invalid global settings/boot flag/callback pointer.");
		tlvBeginOk = false;
		return;
	}

	globalSettingsPtr = globalStore;
	presetsPtr        = presets;
	banksPtr          = NULL;   // banks are raw-only; unused in tagged mode
	bootFlagPtr       = bootFlag;

	tlvMode              = true;
	tlvGlobalSerialize   = gSerialize;
	tlvGlobalDeserialize = gDeserialize;
	tlvPresetSerialize   = pSerialize;
	tlvPresetDeserialize = pDeserialize;
	tlvCtx               = ctx;
	tlvBeginOk           = true;

	ESP_LOGI(TAG, "Mounting storage...");
	tlvMounted = esp32Settings_Mount();

	// Pre-fill defaults so any tag absent from an older file keeps its default, then decode
	// the stored global tags over the top. Deferred: if the mount failed (or the file is
	// missing/corrupt), Phase 2 routes to NewDeviceConfig — we do NOT reconfigure here, so
	// the caller can bring up the display first and show the reset message during the format.
	if (assignDefaultGlobalSettings != NULL)
		assignDefaultGlobalSettings();
	if (tlvMounted) {
		ESP_LOGI(TAG, "Validating stored global settings (tagged)...");
		tlvGlobalStatus = esp32SettingsTlv_LoadFile(GLOBAL_PATH, gDeserialize, ctx);
	} else {
		ESP_LOGE(TAG, "Storage not mounted — deferring reconfigure to phase 2.");
		tlvGlobalStatus = SETTINGS_TLV_MISSING;   // forces reconfigure in Finish
	}
}

// ---------------------------------------------------------------------------
// Per-preset files
// ---------------------------------------------------------------------------
void esp32Settings_UsePresetFiles(uint16_t numPresets, uint16_t perFile,
                                  settings_item_serialize_fn serialize,
                                  settings_item_deserialize_fn deserialize,
                                  void (*defaultOne)(uint16_t index))
{
	if (numPresets == 0 || perFile == 0 || perFile > PRESETS_PER_FILE_MAX ||
	    serialize == NULL || deserialize == NULL || defaultOne == NULL) {
		ESP_LOGE(TAG, "UsePresetFiles: invalid arguments — staying on the single preset file.");
		return;
	}
	presetFiles      = true;
	presetItemCount  = numPresets;
	presetsPerFile   = perFile;
	presetGroupCount = (uint16_t)((numPresets + perFile - 1) / perFile);
	itemSerialize    = serialize;
	itemDeserialize  = deserialize;
	defaultOnePreset = defaultOne;
}

bool esp32Settings_SavePreset(uint16_t index)
{
	if (!presetFiles || presetsPtr == NULL || index >= presetItemCount) {
		ESP_LOGE(TAG, "SavePreset(%u): preset files not in use or index out of range.", (unsigned)index);
		return false;
	}
	// Rewrites the preset's whole group from RAM (RAM is authoritative for its neighbours too).
	return presetGroupSave(index / presetsPerFile);
}

bool esp32Settings_ReadPreset(uint16_t index)
{
	if (!presetFiles || presetsPtr == NULL || index >= presetItemCount) {
		ESP_LOGE(TAG, "ReadPreset(%u): preset files not in use or index out of range.", (unsigned)index);
		return false;
	}
	sOnlyIndex = index;
	bool ok = presetGroupRead(index / presetsPerFile);
	sOnlyIndex = UINT16_MAX;
	return ok;
}

// Read every group file; with `heal`, rewrite any that were missing/corrupt/incomplete so the
// warning does not repeat on every boot.
static void esp32Settings_ReadPresetFiles(bool heal)
{
	uint16_t bad = 0;
	sGroupFileMissing = false;
	for (uint16_t g = 0; g < presetGroupCount; g++) {
		if (!presetGroupRead(g)) {
			bad++;
			if (heal)
				presetGroupSave(g);
		}
	}
	if (heal && sGroupFileMissing)
		presetDirPrune();
	if (bad)
		ESP_LOGW(TAG, "%u of %u preset file(s) missing/invalid — reset to defaults%s.",
		         (unsigned)bad, (unsigned)presetGroupCount, heal ? " and rewritten" : "");
	else
		ESP_LOGI(TAG, "Loaded %u presets from %u files.", (unsigned)presetItemCount, (unsigned)presetGroupCount);
}

// Phase 2: load presets + run the reconfigure decision. May format the filesystem and
// reboot (esp32Settings_NewDeviceConfig). Returns the boot flag value as read from storage.
uint8_t esp32Settings_BootFinishTagged(void)
{
	ESP_LOGI(TAG, "Tagged boot check (phase 2: presets + reconfigure).");
	if (!tlvBeginOk || bootFlagPtr == NULL) {
		ESP_LOGE(TAG, "BootFinishTagged called without a successful BootBeginTagged.");
		return 0;
	}

	settings_tlv_status_t pStatus = SETTINGS_TLV_OK;
	if (presetFiles) {
		// Skipped when the global load failed: that reconfigures (format + rewrite) anyway.
		if (tlvMounted && presetsPtr != NULL && tlvGlobalStatus == SETTINGS_TLV_OK)
			esp32Settings_ReadPresetFiles(true);
	} else if (tlvMounted && presetsPtr != NULL && tlvPresetDeserialize != NULL) {
		if (assignDefaultPresetSettings != NULL)
			assignDefaultPresetSettings();
		pStatus = esp32SettingsTlv_LoadFile(PRESETS_PATH, tlvPresetDeserialize, tlvCtx);
	}

	// A decodable payload (OK) is kept even if some tags were missing/defaulted.
	// Anything else — not mounted, missing file, non-TLV header, or corruption — reconfigures.
	if (tlvGlobalStatus != SETTINGS_TLV_OK || pStatus != SETTINGS_TLV_OK) {
		ESP_LOGI(TAG, "Stored settings missing/invalid (global=%d, presets=%d) — reconfiguring.",
		         (int)tlvGlobalStatus, (int)pStatus);
		esp32Settings_NewDeviceConfig();
	}

	uint8_t bootFlagValue = *bootFlagPtr;
	if (*bootFlagPtr != DEVICE_CONFIGURED_VALUE && *bootFlagPtr != DEVICE_FRESH_RESET_VALUE) {
		ESP_LOGI(TAG, "Configuring new device...");
		esp32Settings_NewDeviceConfig();
	} else {
		ESP_LOGI(TAG, "Tagged standard boot complete.");
		esp32Settings_ListDir(SETTINGS_BASE_PATH, 1);
	}
	return bootFlagValue;
}

// Backward-compatible single-call boot check: Begin immediately followed by Finish.
uint8_t esp32Settings_BootCheckTagged(
		void *globalStore, settings_serialize_fn gSerialize, settings_deserialize_fn gDeserialize,
		void *presets, settings_serialize_fn pSerialize, settings_deserialize_fn pDeserialize,
		uint8_t *bootFlag, void *ctx)
{
	esp32Settings_BootBeginTagged(globalStore, gSerialize, gDeserialize,
	                              presets, pSerialize, pDeserialize, bootFlag, ctx);
	return esp32Settings_BootFinishTagged();
}

void esp32Settings_SetFactoryResetNotify(void (*cb)(void))
{
	factoryResetNotify = cb;
}

// Configures the device to a factory state
void esp32Settings_NewDeviceConfig()
{
	// Surface the reset on the display (if a hook is registered) before the blocking
	// format below — the callback is expected to render a message and return promptly.
	if (factoryResetNotify != NULL)
		factoryResetNotify();

	// Configure default values for global settings
	if (assignDefaultGlobalSettings != NULL)
		assignDefaultGlobalSettings();
	else
		ESP_LOGE(TAG, "No default global settings function assigned. Pointer is null.");

	// Set the boot flag to indicate the device has been configured. FRESH (not CONFIGURED) so
	// the app can tell "first boot since a reconfigure" apart from a normal boot; both are
	// accepted as configured by the boot checks, and the app promotes it when it chooses.
	*bootFlagPtr = DEVICE_FRESH_RESET_VALUE;

	// Format the file system to revert to a clean state
	ESP_LOGI(TAG, "Formatting file system...");
	esp_err_t err = esp_littlefs_format(SETTINGS_PARTITION_LABEL);
	if (err != ESP_OK)
		ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(err));

	// Create the storage file for the global config
	ESP_LOGI(TAG, "Creating global settings file...");
	esp32Settings_SaveGlobalSettings();

	ESP_LOGI(TAG, "Boot flag = %d", *bootFlagPtr);

	// Configure default bank values (skipped when banks are unused)
	if (banksPtr == NULL) {
		ESP_LOGI(TAG, "Banks not used (NULL) — skipping default bank config.");
	} else if (assignDefaultBankSettings != NULL) {
		assignDefaultBankSettings();
	} else {
		ESP_LOGE(TAG, "No default bank settings function assigned. Pointer is null.");
	}

	// Create the banks storage file (self-skips when unused)
	ESP_LOGI(TAG, "Creating banks file...");
	esp32Settings_SaveBanks();

	// Configure default preset values (skipped when presets are unused)
	if (presetsPtr == NULL) {
		ESP_LOGI(TAG, "Presets not used (NULL) — skipping default preset config.");
	} else if (assignDefaultPresetSettings != NULL) {
		assignDefaultPresetSettings();
	} else {
		ESP_LOGE(TAG, "No default preset settings function assigned. Pointer is null.");
	}

	// Create the presets storage file (self-skips when unused)
	ESP_LOGI(TAG, "Creating presets file...");
	esp32Settings_SavePresets();

	ESP_LOGI(TAG, "Device configured. Rebooting.");

	vTaskDelay(pdMS_TO_TICKS(10));
	esp32Settings_SoftwareReset();
}

void esp32Settings_StandardBoot()
{
	// Re-read the persisted data into the struct pointers (bank/preset self-skip if unused)
	esp32Settings_ReadGlobalSettings();
	esp32Settings_ReadBanks();
	esp32Settings_ReadPresets();

	ESP_LOGI(TAG, "Standard boot complete!");
	esp32Settings_ListDir(SETTINGS_BASE_PATH, 1);
}

void esp32Settings_SoftwareReset()
{
	esp_restart();
}

void esp32Settings_AssignDefaultGlobalSettings(void (*fptr)())
{
	if (fptr != NULL)
		assignDefaultGlobalSettings = fptr;
	else
		ESP_LOGE(TAG, "No default global settings function assigned. Pointer is null.");
}

void esp32Settings_AssignDefaultBankSettings(void (*fptr)())
{
	if (fptr != NULL)
		assignDefaultBankSettings = fptr;
	else
		ESP_LOGE(TAG, "No default bank settings function assigned. Pointer is null.");
}

void esp32Settings_AssignDefaultPresetSettings(void (*fptr)())
{
	if (fptr != NULL)
		assignDefaultPresetSettings = fptr;
	else
		ESP_LOGE(TAG, "No default preset settings function assigned. Pointer is null.");
}

// Invalidate the stored config so the next boot reconfigures to defaults, by DELETING
// the global settings file (works for both the raw and tagged paths — a missing global
// file routes BootCheck/BootFinishTagged to esp32Settings_NewDeviceConfig, which formats
// and rewrites defaults at early boot where heap is plentiful).
//
// We deliberately do NOT rewrite the file with a cleared boot flag: that full serialize is
// heap-hungry, and running from the Device-API task with BLE/USB/LVGL up on the no-PSRAM N8
// it was starving the littlefs write ("Failed to reserve header"), silently leaving the old
// CONFIGURED file in place so the factory reset did nothing. remove() needs almost no heap.
void esp32Settings_ResetAllSettings()
{
	ESP_LOGI(TAG, "Factory reset: deleting %s to force reconfigure on next boot.", GLOBAL_PATH);
	if (remove(GLOBAL_PATH) != 0)
		ESP_LOGW(TAG, "Could not delete %s (may already be absent) - reconfigure still expected.", GLOBAL_PATH);
	// littlefs commits the unlink synchronously, so no settle is needed for durability;
	// the short delay just lets the UART log flush before the hard restart.
	vTaskDelay(pdMS_TO_TICKS(50));
	esp32Settings_SoftwareReset();
}

void esp32Settings_ReadGlobalSettings()
{
	ESP_LOGI(TAG, "Reading global settings...");
	if (tlvMode) {
		if (assignDefaultGlobalSettings != NULL)
			assignDefaultGlobalSettings();  // defaults under any tags absent from the file
		if (esp32SettingsTlv_LoadFile(GLOBAL_PATH, tlvGlobalDeserialize, tlvCtx) != SETTINGS_TLV_OK)
			ESP_LOGE(TAG, "Global settings (TLV) read failed.");
		return;
	}
	if (!esp32Settings_ReadBlob(GLOBAL_PATH, globalSettingsPtr, globalSettingsSize))
		ESP_LOGE(TAG, "Global settings read failed.");
}

void esp32Settings_SaveGlobalSettings()
{
	ESP_LOGI(TAG, "Saving global settings to file.");
	if (tlvMode) {
		if (!esp32SettingsTlv_SaveFile(GLOBAL_PATH, GLOBAL_TMP_PATH, tlvGlobalSerialize, tlvCtx))
			ESP_LOGE(TAG, "Global settings (TLV) save failed.");
		return;
	}
	esp32Settings_WriteBlob(GLOBAL_PATH, GLOBAL_TMP_PATH, globalSettingsPtr, globalSettingsSize);
}

void esp32Settings_ReadBanks()
{
	if (banksPtr == NULL) {
		ESP_LOGI(TAG, "Banks not used (NULL) — skipping read.");
		return;
	}
	ESP_LOGI(TAG, "Reading banks...");
	if (!esp32Settings_ReadBlob(BANKS_PATH, banksPtr, (size_t)bankSize * numBanks))
		ESP_LOGE(TAG, "Banks read failed.");
}

void esp32Settings_SaveBanks()
{
	if (banksPtr == NULL) {
		ESP_LOGI(TAG, "Banks not used (NULL) — skipping save.");
		return;
	}
	ESP_LOGI(TAG, "Saving banks to file.");
	esp32Settings_WriteBlob(BANKS_PATH, BANKS_TMP_PATH, banksPtr,
							(size_t)bankSize * numBanks);
}

void esp32Settings_ReadPresets()
{
	if (presetsPtr == NULL) {
		ESP_LOGI(TAG, "Presets not used (NULL) — skipping read.");
		return;
	}
	ESP_LOGI(TAG, "Reading presets...");
	if (tlvMode && presetFiles) {
		esp32Settings_ReadPresetFiles(false);
		return;
	}
	if (tlvMode) {
		if (tlvPresetDeserialize == NULL) {
			ESP_LOGE(TAG, "No preset deserialiser in tagged mode.");
			return;
		}
		if (assignDefaultPresetSettings != NULL)
			assignDefaultPresetSettings();
		if (esp32SettingsTlv_LoadFile(PRESETS_PATH, tlvPresetDeserialize, tlvCtx) != SETTINGS_TLV_OK)
			ESP_LOGE(TAG, "Presets (TLV) read failed.");
		return;
	}
	if (!esp32Settings_ReadBlob(PRESETS_PATH, presetsPtr, (size_t)presetSize * numPresets))
		ESP_LOGE(TAG, "Presets read failed.");
}

void esp32Settings_SavePresets()
{
	if (presetsPtr == NULL) {
		ESP_LOGI(TAG, "Presets not used (NULL) — skipping save.");
		return;
	}
	ESP_LOGI(TAG, "Saving presets to file.");
	if (tlvMode && presetFiles) {
		uint16_t failed = 0;
		for (uint16_t g = 0; g < presetGroupCount; g++)
			if (!presetGroupSave(g))
				failed++;
		if (failed)
			ESP_LOGE(TAG, "Presets save: %u of %u file(s) failed.", (unsigned)failed, (unsigned)presetGroupCount);
		else
			ESP_LOGI(TAG, "Saved %u presets to %u files.", (unsigned)presetItemCount, (unsigned)presetGroupCount);
		return;
	}
	if (tlvMode) {
		if (tlvPresetSerialize == NULL) {
			ESP_LOGE(TAG, "No preset serialiser in tagged mode.");
			return;
		}
		if (!esp32SettingsTlv_SaveFile(PRESETS_PATH, PRESETS_TMP_PATH, tlvPresetSerialize, tlvCtx))
			ESP_LOGE(TAG, "Presets (TLV) save failed.");
		return;
	}
	esp32Settings_WriteBlob(PRESETS_PATH, PRESETS_TMP_PATH, presetsPtr,
							(size_t)presetSize * numPresets);
}

// Diagnostic directory listing (VERBOSE only) via POSIX dirent over the VFS mount.
static void esp32Settings_ListDir(const char *dirname, uint8_t levels)
{
	// The walk itself is expensive — two littlefs stat() calls per entry, each a linear
	// directory search — so skip it entirely unless its VERBOSE output can actually print.
	// With per-preset files (128 entries) it otherwise ran ~4 s on the boot path without
	// yielding and tripped the task watchdog (IDLE0 starved), even with the output compiled out.
#if LOG_LOCAL_LEVEL < ESP_LOG_VERBOSE
	(void)dirname;
	(void)levels;
	return;
#else
	if (esp_log_level_get(TAG) < ESP_LOG_VERBOSE)
		return;
	ESP_LOGV(TAG, "Listing directory: %s", dirname);

	DIR *dir = opendir(dirname);
	if (dir == NULL) {
		ESP_LOGV(TAG, "- failed to open directory");
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		char path[300];
		sprintf(path, "%s/%s", dirname, entry->d_name);

		struct stat st;
		bool is_dir = (entry->d_type == DT_DIR) ||
					  (stat(path, &st) == 0 && S_ISDIR(st.st_mode));
		if (is_dir) {
			ESP_LOGV(TAG, "  DIR : %s", entry->d_name);
			if (levels)
				esp32Settings_ListDir(path, levels - 1);
		} else {
			long size = (stat(path, &st) == 0) ? (long)st.st_size : -1;
			ESP_LOGV(TAG, "  FILE: %s (%ld)", entry->d_name, size);
		}
	}
	closedir(dir);
	ESP_LOGV(TAG, "directory listing complete.");
#endif
}
