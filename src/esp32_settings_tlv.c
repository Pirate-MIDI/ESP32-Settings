#include "esp32_settings_tlv.h"

#include <string.h>

#include "esp_log.h"
#include "esp_rom_crc.h"   // esp_rom_crc32_le (chainable via its seed argument)

static const char *TAG = "ESP32_SETTINGS_TLV";

// Small stack scratch used to drain skipped/unknown bytes while still feeding
// them to the running CRC. Kept tiny — no PSRAM on the N8 target.
#define TLV_SCRATCH 64

// ===========================================================================
// Writer
// ===========================================================================

bool esp32SettingsTlv_BeginWrite(tlv_writer_t *w, const char *tmp_path)
{
	memset(w, 0, sizeof(*w));
	w->f = fopen(tmp_path, "wb");
	if (w->f == NULL) {
		ESP_LOGE(TAG, "Failed to open %s for write", tmp_path);
		return false;
	}
	// Reserve the header — its length + CRC are unknown until the payload is
	// streamed, so write a placeholder now and back-patch it in FinishWrite.
	settings_header_t placeholder = {0};
	if (fwrite(&placeholder, 1, sizeof placeholder, w->f) != sizeof placeholder) {
		ESP_LOGE(TAG, "Failed to reserve header in %s", tmp_path);
		fclose(w->f);
		w->f = NULL;
		return false;
	}
	w->ok = true;
	return true;
}

void esp32SettingsTlv_BeginCount(tlv_writer_t *w)
{
	memset(w, 0, sizeof(*w));
	w->ok = true;
	w->count = true;
}

void esp32SettingsTlv_WriteRaw(tlv_writer_t *w, const void *data, uint16_t len)
{
	if (!w->ok)
		return;
	if (w->count) {
		w->len += len;
		return;
	}
	if (w->f == NULL)
		return;
	if (fwrite(data, 1, len, w->f) != len) {
		// Almost always heap exhaustion in the caller starving the littlefs
		// block write (see docs/SOLUTIONS.md), not a full filesystem.
		ESP_LOGE(TAG, "Short TLV write (%u bytes) — low heap?", (unsigned)len);
		w->ok = false;
		return;
	}
	w->crc = esp_rom_crc32_le(w->crc, (const uint8_t *)data, len);
	w->len += len;
}

void esp32SettingsTlv_WriteRecordHeader(tlv_writer_t *w, uint16_t tag, uint16_t len)
{
	uint8_t hdr[4];
	memcpy(&hdr[0], &tag, sizeof tag);
	memcpy(&hdr[2], &len, sizeof len);
	esp32SettingsTlv_WriteRaw(w, hdr, sizeof hdr);
}

void esp32SettingsTlv_WriteRecord(tlv_writer_t *w, uint16_t tag, const void *data, uint16_t len)
{
	esp32SettingsTlv_WriteRecordHeader(w, tag, len);
	esp32SettingsTlv_WriteRaw(w, data, len);
}

void esp32SettingsTlv_WriteFields(tlv_writer_t *w, const void *base,
                                  const tlv_field_t *fields, size_t n)
{
	for (size_t i = 0; i < n; i++)
		esp32SettingsTlv_WriteRecord(w, fields[i].tag,
		                             (const uint8_t *)base + fields[i].offset,
		                             fields[i].size);
}

bool esp32SettingsTlv_FinishWrite(tlv_writer_t *w, const char *path, const char *tmp_path)
{
	if (w->f == NULL)
		return false;

	if (!w->ok) {
		fclose(w->f);
		w->f = NULL;
		remove(tmp_path);
		return false;
	}

	settings_header_t hdr = {
		.magic = SETTINGS_MAGIC,
		.version = SETTINGS_TLV_VERSION,
		.reserved = 0,
		.length = w->len,
		.crc32 = w->crc,
	};
	bool ok = (fseek(w->f, 0, SEEK_SET) == 0) &&
	          (fwrite(&hdr, 1, sizeof hdr, w->f) == sizeof hdr);
	fclose(w->f);
	w->f = NULL;

	if (!ok) {
		ESP_LOGE(TAG, "Failed to back-patch header in %s", tmp_path);
		remove(tmp_path);
		return false;
	}
	if (rename(tmp_path, path) != 0) {
		ESP_LOGE(TAG, "rename %s -> %s failed", tmp_path, path);
		remove(tmp_path);
		return false;
	}
	ESP_LOGI(TAG, "Wrote %u payload bytes to %s", (unsigned)w->len, path);
	return true;
}

bool esp32SettingsTlv_SaveFile(const char *path, const char *tmp_path,
                               settings_serialize_fn serialize, void *ctx)
{
	tlv_writer_t w;
	if (!esp32SettingsTlv_BeginWrite(&w, tmp_path))
		return false;
	if (!serialize(&w, ctx))
		w.ok = false;
	return esp32SettingsTlv_FinishWrite(&w, path, tmp_path);
}

// ===========================================================================
// Reader
// ===========================================================================

// Consume + CRC up to `n` bytes of the current record's remaining value,
// copying the first min(n, dst_size) into dst (dst may be NULL to discard).
static uint16_t tlv_consume(tlv_reader_t *r, void *dst, uint16_t dst_size, uint16_t n)
{
	uint16_t remaining = r->cur_len - r->cur_read;
	if (n > remaining)
		n = remaining;

	uint16_t copied = 0;
	uint8_t scratch[TLV_SCRATCH];
	uint16_t left = n;
	while (left > 0) {
		uint16_t want = (left < TLV_SCRATCH) ? left : TLV_SCRATCH;
		// Read straight into dst while there's room for it, else into scratch.
		uint8_t *into = (dst != NULL && copied < dst_size) ? (uint8_t *)dst + copied : scratch;
		if (into != scratch) {
			uint16_t room = dst_size - copied;
			if (want > room)
				want = room;
		}
		if (fread(into, 1, want, r->f) != want) {
			r->err = true;
			return copied;
		}
		r->crc = esp_rom_crc32_le(r->crc, into, want);
		r->consumed += want;
		r->cur_read += want;
		if (into != scratch)
			copied += want;
		left -= want;
	}
	return copied;
}

settings_tlv_status_t esp32SettingsTlv_BeginRead(tlv_reader_t *r, const char *path)
{
	memset(r, 0, sizeof(*r));
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		ESP_LOGW(TAG, "%s not present", path);
		return SETTINGS_TLV_MISSING;
	}

	settings_header_t hdr;
	if (fread(&hdr, 1, sizeof hdr, f) != sizeof hdr ||
	    hdr.magic != SETTINGS_MAGIC ||
	    hdr.version != SETTINGS_TLV_VERSION) {
		ESP_LOGW(TAG, "%s invalid header (magic/version) — not a TLV file", path);
		fclose(f);
		return SETTINGS_TLV_BAD_HEADER;
	}

	r->f = f;
	r->payload_len = hdr.length;
	r->stored_crc = hdr.crc32;
	return SETTINGS_TLV_OK;
}

bool esp32SettingsTlv_NextRecord(tlv_reader_t *r, uint16_t *tag, uint16_t *len)
{
	if (r->err || r->f == NULL)
		return false;

	// Drain any bytes of the previous record the caller didn't read (this is how
	// an unknown tag gets skipped) so the CRC still covers the whole payload.
	if (r->cur_read < r->cur_len)
		tlv_consume(r, NULL, 0, r->cur_len - r->cur_read);
	if (r->err)
		return false;

	// Inside a container (EnterRecord) the stream ends at its limit, not the payload's.
	uint32_t end = r->limit ? r->limit : r->payload_len;
	if (r->consumed >= end)
		return false; // clean end of payload / container

	uint8_t hdr[4];
	if (end - r->consumed < sizeof hdr ||
	    fread(hdr, 1, sizeof hdr, r->f) != sizeof hdr) {
		r->err = true;
		return false;
	}
	r->crc = esp_rom_crc32_le(r->crc, hdr, sizeof hdr);
	r->consumed += sizeof hdr;

	uint16_t t, l;
	memcpy(&t, &hdr[0], sizeof t);
	memcpy(&l, &hdr[2], sizeof l);
	if ((uint32_t)l > end - r->consumed) {
		r->err = true; // record claims more bytes than the payload / container holds
		return false;
	}
	r->cur_len = l;
	r->cur_read = 0;
	if (tag) *tag = t;
	if (len) *len = l;
	return true;
}

bool esp32SettingsTlv_ReadFieldExact(tlv_reader_t *r, void *dst, uint16_t expected)
{
	if (r->cur_read == 0 && r->cur_len == expected) {
		tlv_consume(r, dst, expected, expected);
		return !r->err;
	}
	// Size changed (or already partly read) — keep the caller's default.
	tlv_consume(r, NULL, 0, r->cur_len - r->cur_read);
	return false;
}

uint16_t esp32SettingsTlv_ReadRaw(tlv_reader_t *r, void *dst, uint16_t n)
{
	return tlv_consume(r, dst, n, n);
}

void esp32SettingsTlv_SkipRaw(tlv_reader_t *r, uint16_t n)
{
	tlv_consume(r, NULL, 0, n);
}

uint16_t esp32SettingsTlv_RecordRemaining(const tlv_reader_t *r)
{
	return r->cur_len - r->cur_read;
}

void esp32SettingsTlv_EnterRecord(tlv_reader_t *r)
{
	if (r->err || r->limit != 0) {
		r->err = true;   // one nesting level only
		return;
	}
	r->limit = r->consumed + (uint32_t)(r->cur_len - r->cur_read);
	r->cur_len = 0;
	r->cur_read = 0;
}

void esp32SettingsTlv_ExitRecord(tlv_reader_t *r)
{
	if (r->limit == 0)
		return;
	if (!r->err) {
		// Drain the unread tail of the current inner record, then any inner records left.
		if (r->cur_read < r->cur_len)
			tlv_consume(r, NULL, 0, r->cur_len - r->cur_read);
		if (!r->err && r->consumed < r->limit) {
			r->cur_len = (uint16_t)(r->limit - r->consumed);   // container value is <= 0xFFFF
			r->cur_read = 0;
			tlv_consume(r, NULL, 0, r->cur_len);
		}
		if (r->consumed != r->limit)
			r->err = true;
	}
	r->limit = 0;
	r->cur_len = 0;
	r->cur_read = 0;
}

bool esp32SettingsTlv_FinishRead(tlv_reader_t *r)
{
	bool ok = false;
	if (r->f != NULL) {
		ok = !r->err &&
		     (r->consumed == r->payload_len) &&
		     (r->crc == r->stored_crc);
		if (!ok)
			ESP_LOGW(TAG, "TLV payload invalid (consumed %u/%u, CRC %s)",
			         (unsigned)r->consumed, (unsigned)r->payload_len,
			         (r->crc == r->stored_crc) ? "ok" : "mismatch");
		fclose(r->f);
		r->f = NULL;
	}
	return ok;
}

const tlv_field_t *esp32SettingsTlv_FindField(const tlv_field_t *fields, size_t n, uint16_t tag)
{
	for (size_t i = 0; i < n; i++)
		if (fields[i].tag == tag)
			return &fields[i];
	return NULL;
}

settings_tlv_status_t esp32SettingsTlv_LoadFile(const char *path,
                                                settings_deserialize_fn deserialize, void *ctx)
{
	tlv_reader_t r;
	settings_tlv_status_t status = esp32SettingsTlv_BeginRead(&r, path);
	if (status != SETTINGS_TLV_OK)
		return status;

	bool desOk = deserialize(&r, ctx);
	if (!desOk) {
		if (r.f != NULL)
			fclose(r.f);
		return SETTINGS_TLV_IO_ERROR;
	}
	if (!esp32SettingsTlv_FinishRead(&r))
		return SETTINGS_TLV_BAD_CRC;
	return SETTINGS_TLV_OK;
}
