#ifndef ESP32_SETTINGS_H
#define ESP32_SETTINGS_H

#include <stdint.h>
#include <stddef.h>

#include "esp32_settings_tlv.h"   // TLV codec + settings_serialize_fn / _deserialize_fn

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_FIRST_BOOT_VALUE 100
#define DEVICE_CONFIGURED_VALUE 114 // Arbitrary value to indicate the device has been configured
// Written by esp32Settings_NewDeviceConfig() instead of DEVICE_CONFIGURED_VALUE: "configured,
// but this is the first boot since a reconfigure". Both boot checks accept it as configured.
// The app may promote it to DEVICE_CONFIGURED_VALUE (and save) once it has seen whatever
// triggered the reset go away — Scribble uses it to fire its switch-1 hold-at-boot factory
// reset exactly once per hold, no matter how long the switch stays down.
#define DEVICE_FRESH_RESET_VALUE 115

// `banks` and/or `presets` may be NULL to indicate that storage element is unused:
// it is then ignored entirely (no size validation, existence check, load or save).
// `globalStore` and `bootFlag` are always required.
// NOTE: the first param is named `globalStore`, not `globalSettings`, on purpose —
// some consumers `#define globalSettings (*ptr)` (PSRAM offload), which would expand
// inside this declaration and corrupt the signature.
uint8_t esp32Settings_BootCheck(	void* globalStore, uint16_t gSize,
										void* banks, uint16_t bSize, size_t numBanks,
										void* presets, uint16_t pSize, size_t numPresets,
										uint8_t* bootFlag);

// Forward-compatible (TLV) boot check. Decodes tagged streams via the caller's
// (de)serialise callbacks instead of validating a raw struct blob by length, so
// evolving GlobalSettings/Preset never wipes user settings. Register the
// default-assignment callbacks first (esp32Settings_AssignDefault*); banks are not
// supported in tagged mode. `presets` and its callbacks may be NULL if unused.
// See esp32_settings_tlv.h and docs/settings-migration-plan.md.
uint8_t esp32Settings_BootCheckTagged(
		void* globalStore, settings_serialize_fn gSerialize, settings_deserialize_fn gDeserialize,
		void* presets, settings_serialize_fn pSerialize, settings_deserialize_fn pDeserialize,
		uint8_t* bootFlag, void* ctx);

// Phased tagged boot check. Splits esp32Settings_BootCheckTagged so early-boot code can
// run between the two halves. Phase 1 (Begin) mounts storage and loads the global settings
// only — so the caller can read e.g. display rotation and bring up a boot screen BEFORE the
// slower preset load or any factory reconfigure. Phase 2 (Finish) loads presets and performs
// the reconfigure decision (which may format the filesystem and reboot). Call order:
//   esp32Settings_BootBeginTagged(...);  // global valid after this
//   ...early work (display, etc.)...
//   esp32Settings_BootFinishTagged();
// Same argument contract as esp32Settings_BootCheckTagged, which is now a Begin+Finish wrapper.
void esp32Settings_BootBeginTagged(
		void* globalStore, settings_serialize_fn gSerialize, settings_deserialize_fn gDeserialize,
		void* presets, settings_serialize_fn pSerialize, settings_deserialize_fn pDeserialize,
		uint8_t* bootFlag, void* ctx);
uint8_t esp32Settings_BootFinishTagged(void);

// ---------------------------------------------------------------------------
// Grouped preset files (tagged mode only, optional).
//
// Call BEFORE esp32Settings_BootBeginTagged. Presets are then stored `perFile` to a TLV file
// (/lfs/presets/gNNN.tlv, own header + CRC, written atomically via tmp + rename), one
// container record per preset, instead of one blob holding every preset, so:
//   - one preset can be saved on its own (esp32Settings_SavePreset rewrites its group);
//   - a missing or corrupt file resets ONLY that group's presets to defaults (via
//     defaultOne) and is rewritten — preset data never triggers a factory reconfigure;
//   - esp32Settings_SavePresets / ReadPresets still cover all of them.
// Grouping exists for boot time: each littlefs open costs a directory lookup, so 128 single
// files took ~2.2 s to load on N8 (beta.2). perFile 1..32.
// The single-blob presets file of the default layout is not read in this mode (no migration);
// pass NULL preset (de)serialisers to BootBeginTagged.
// The per-item callbacks write/read ONE preset's records (tags need no index). The serialiser
// runs twice per save (sizing pass, then write), so it must only read RAM.
// Not thread-safe: save/read presets from one task at a time.
// ---------------------------------------------------------------------------
typedef bool (*settings_item_serialize_fn)(tlv_writer_t *w, uint16_t index, void *ctx);
typedef bool (*settings_item_deserialize_fn)(tlv_reader_t *r, uint16_t index, void *ctx);

void esp32Settings_UsePresetFiles(uint16_t numPresets, uint16_t perFile,
                                  settings_item_serialize_fn serialize,
                                  settings_item_deserialize_fn deserialize,
                                  void (*defaultOne)(uint16_t index));
// Per-preset-file mode only. Return false on an I/O failure / out-of-range index (Read:
// the preset has been reset to defaults in that case).
bool esp32Settings_SavePreset(uint16_t index);
bool esp32Settings_ReadPreset(uint16_t index);

// Optional hook invoked once, immediately before a factory reconfigure formats the
// filesystem (from esp32Settings_NewDeviceConfig). Lets the app surface a "resetting"
// message on an already-visible display before the blocking format. NULL = no callback.
void esp32Settings_SetFactoryResetNotify(void (*cb)(void));

void esp32Settings_NewDeviceConfig();
void esp32Settings_StandardBoot();
void esp32Settings_SoftwareReset();

// Settings callbacks
void esp32Settings_AssignDefaultGlobalSettings(void (*fptr)());
void esp32Settings_AssignDefaultBankSettings(void (*fptr)());
void esp32Settings_AssignDefaultPresetSettings(void (*fptr)());
void esp32Settings_ResetAllSettings();
void esp32Settings_ReadGlobalSettings();
void esp32Settings_SaveGlobalSettings();
void esp32Settings_ReadBanks();
void esp32Settings_SaveBanks();
void esp32Settings_ReadPresets();
void esp32Settings_SavePresets();

#ifdef __cplusplus
}
#endif

#endif // ESP32_SETTINGS_H
