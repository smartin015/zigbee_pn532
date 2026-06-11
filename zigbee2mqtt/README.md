# zigbee2mqtt External Converter

This directory contains the external converter for the **Zigbee NFC Bridge**
(Xiao ESP32C6 + PN532).

## Installation

### Option A — external_converters directory

Copy the converter into your zigbee2mqtt data directory:

```bash
cp nfc-bridge.js /opt/zigbee2mqtt/data/external_converters/
```

Then ensure `external_converters` is enabled in `configuration.yaml`:

```yaml
external_converters: []
```

(zigbee2mqtt auto-loads all `.js` files from `data/external_converters/`.)

### Option B — explicit path

In `configuration.yaml`:

```yaml
external_converters:
  - /path/to/zigbee_pn532/zigbee2mqtt/nfc-bridge.js
```

### Restart

```bash
systemctl restart zigbee2mqtt
```

## Exposed Entities

| Name | Type | Access | Description |
|---|---|---|---|
| `nfc_text` | text | read | Last-read NFC tag text |
| `nfc_tag_uid` | text | read | UID of the last-read tag (hex) |
| `nfc_pending_write` | text | read/write | Text to write to the next presented tag (empty string cancels) |
| `nfc_reader_present` | binary | read | `true` when PN532 module is reachable |
| `nfc_auth_enabled` | binary | read/write | Enable NTAG password authentication |
| `nfc_auth_pwd` | text | read/write | NTAG authentication password (4 bytes, hex) |
| `nfc_auth_pack` | text | read/write | NTAG password acknowledge (2 bytes, hex) |
| `nfc_last_read_ts` | text | read | ISO 8601 timestamp of the last successful tag read (e.g. "2025-01-15T10:30:45Z") |
| `nfc_last_write_ts` | text | read | ISO 8601 timestamp of the last successful tag write |

## Usage

- **Write a tag**: set `nfc_pending_write` to the desired text, then present a
  tag to the PN532. The bridge writes the NDEF record and clears the attribute.
- **Read a tag**: present a tag — `nfc_text` and `nfc_tag_uid` update
  automatically via attribute reporting.
- **Hot-plug**: `nfc_reader_present` tracks whether the PN532 module is
  physically connected.
