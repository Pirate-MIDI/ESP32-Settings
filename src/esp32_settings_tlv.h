#ifndef ESP32_SETTINGS_TLV_H
#define ESP32_SETTINGS_TLV_H

// ---------------------------------------------------------------------------
// Tagged (TLV) settings codec — forward-compatible persistence.
//
// Replaces the raw-struct-blob format with a self-describing stream of
// [u16 tag][u16 len][len bytes value] records. Load matches by tag, skips
// unknown tags, and leaves the caller's pre-filled default for any missing
// tag. Adding / removing / reordering a struct field therefore never wipes a
// user's settings and needs no per-version migration code — see
// docs/settings-migration-plan.md ("Decision (locked 2026-08-22)").
//
// The on-disk 16-byte integrity header still wraps the whole record stream, so
// a corrupt/truncated file (bad magic/version/length/CRC) is detected and the
// caller falls back to defaults exactly as before.
//
// I/O is streamed record-by-record with an incrementally-chained CRC (no large
// transient buffer — the N8 target has no PSRAM). On write the header length +
// CRC are back-patched after the payload is streamed; on read the CRC is
// verified after the whole payload has been consumed.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared on-disk integrity header (also used by the legacy raw path in
// esp32_settings.c). CRC32 is over the payload only, not this header.
#define SETTINGS_MAGIC        0x53455454u  // 'SETT'
#define SETTINGS_TLV_VERSION  2            // version==2 in the header => TLV payload

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t version;
	uint16_t reserved;  // keeps the payload 4-byte aligned in the file
	uint32_t length;    // payload length in bytes
	uint32_t crc32;     // esp_rom_crc32_le(0, payload, length)
} settings_header_t;

// Result of opening/loading a TLV file. Only MISSING / BAD_HEADER / BAD_CRC /
// IO_ERROR should route to a factory reconfigure — an OK decode with defaulted
// (missing) tags is a success.
typedef enum {
	SETTINGS_TLV_OK = 0,
	SETTINGS_TLV_MISSING,     // file not present
	SETTINGS_TLV_BAD_HEADER,  // wrong magic/version (e.g. a legacy raw v1 blob)
	SETTINGS_TLV_BAD_CRC,     // length/CRC mismatch — corruption
	SETTINGS_TLV_IO_ERROR,    // read/write/serialise failure
} settings_tlv_status_t;

// A stable-tagged field descriptor. The project layer generates an array of
// these from an X-macro struct field list (via offsetof/sizeof). `tag` is
// permanent — never reuse a retired tag number.
typedef struct {
	uint16_t tag;
	uint16_t size;    // sizeof the member
	uint32_t offset;  // offsetof the member within the struct base
} tlv_field_t;

// ---- Writer -------------------------------------------------------------
typedef struct {
	FILE    *f;
	uint32_t crc;   // running CRC over payload written so far
	uint32_t len;   // payload bytes written so far
	bool     ok;    // cleared on the first short write; makes later calls no-ops
	bool     count; // counting-only writer (BeginCount): no file, no CRC, just `len`
} tlv_writer_t;

// Open <tmp_path> and reserve space for the header (back-patched by FinishWrite).
bool     esp32SettingsTlv_BeginWrite(tlv_writer_t *w, const char *tmp_path);
// Counting-only writer: run a serialiser into it to learn how many payload bytes it emits
// (w->len) without touching storage. Used to size a container record before writing it.
void     esp32SettingsTlv_BeginCount(tlv_writer_t *w);
// Low-level: emit a record header, then raw value bytes (both CRC'd + counted).
void     esp32SettingsTlv_WriteRecordHeader(tlv_writer_t *w, uint16_t tag, uint16_t len);
void     esp32SettingsTlv_WriteRaw(tlv_writer_t *w, const void *data, uint16_t len);
// Convenience: one complete [tag][len][value] record.
void     esp32SettingsTlv_WriteRecord(tlv_writer_t *w, uint16_t tag, const void *data, uint16_t len);
// Emit every field in a descriptor table as its own record.
void     esp32SettingsTlv_WriteFields(tlv_writer_t *w, const void *base,
                                      const tlv_field_t *fields, size_t n);
// Back-patch the header, close, and atomically rename tmp -> path. Aborts
// (removes tmp, returns false) if any prior write failed.
bool     esp32SettingsTlv_FinishWrite(tlv_writer_t *w, const char *path, const char *tmp_path);

// ---- Reader -------------------------------------------------------------
typedef struct {
	FILE    *f;
	uint32_t crc;          // running CRC over payload consumed so far
	uint32_t payload_len;  // from the header
	uint32_t consumed;     // payload bytes read so far
	uint32_t stored_crc;   // from the header
	uint16_t cur_len;      // length of the record currently being read
	uint16_t cur_read;     // bytes of the current record already consumed
	bool     err;          // a short read / corruption was seen
	uint32_t limit;        // EnterRecord: payload offset where the nested stream ends (0 = none)
} tlv_reader_t;

// Open + validate the header. Returns MISSING / BAD_HEADER on failure (file is
// left closed in that case); OK leaves the reader positioned at the first record.
settings_tlv_status_t esp32SettingsTlv_BeginRead(tlv_reader_t *r, const char *path);
// Advance to the next record (draining any unread bytes of the previous one so
// unknown tags are skipped automatically + still CRC'd). Returns false at the
// clean end of the payload, or on a corruption/short read (check FinishRead).
bool     esp32SettingsTlv_NextRecord(tlv_reader_t *r, uint16_t *tag, uint16_t *len);
// Copy the current record's value into dst only if its stored length equals
// `expected` (a scalar/blob whose size is unchanged); otherwise consume + CRC
// the bytes, copy nothing, and return false so the caller keeps its default.
bool     esp32SettingsTlv_ReadFieldExact(tlv_reader_t *r, void *dst, uint16_t expected);
// Read up to `n` bytes of the current record's remaining value into dst.
uint16_t esp32SettingsTlv_ReadRaw(tlv_reader_t *r, void *dst, uint16_t n);
// Consume + CRC up to `n` bytes of the current record without copying them.
void     esp32SettingsTlv_SkipRaw(tlv_reader_t *r, uint16_t n);
// Bytes left in the current record.
uint16_t esp32SettingsTlv_RecordRemaining(const tlv_reader_t *r);
// Container records (one level): EnterRecord treats the rest of the current record's value
// as a nested record stream, so NextRecord returns false at its end instead of the payload's.
// ExitRecord skips whatever the nested reader left unread and resumes the outer stream.
void     esp32SettingsTlv_EnterRecord(tlv_reader_t *r);
void     esp32SettingsTlv_ExitRecord(tlv_reader_t *r);
// Close + verify the whole payload was consumed and the CRC matches. Call once,
// after NextRecord has returned false. Returns false on CRC/length mismatch.
bool     esp32SettingsTlv_FinishRead(tlv_reader_t *r);

// Find a field descriptor by tag (linear scan), or NULL.
const tlv_field_t *esp32SettingsTlv_FindField(const tlv_field_t *fields, size_t n, uint16_t tag);

// ---- File orchestration -------------------------------------------------
// The caller supplies a serialiser (writes records via the writer primitives)
// and a deserialiser (loops NextRecord until it returns false, dispatching each
// tag). `ctx` is passed through untouched.
typedef bool (*settings_serialize_fn)(tlv_writer_t *w, void *ctx);
typedef bool (*settings_deserialize_fn)(tlv_reader_t *r, void *ctx);

bool                  esp32SettingsTlv_SaveFile(const char *path, const char *tmp_path,
                                                settings_serialize_fn serialize, void *ctx);
settings_tlv_status_t esp32SettingsTlv_LoadFile(const char *path,
                                                settings_deserialize_fn deserialize, void *ctx);

#ifdef __cplusplus
}
#endif

#endif // ESP32_SETTINGS_TLV_H
