// zigbee2mqtt external converter for the Zigbee NFC Reader/Writer Bridge
//
// Usage:
//   1. Copy this file into your zigbee2mqtt data directory, e.g.:
//        cp nfc-bridge.js /opt/zigbee2mqtt/data/external_converters/
//   2. Or set the path in zigbee2mqtt configuration.yaml:
//        external_converters:
//          - /path/to/nfc-bridge.js
//   3. Restart zigbee2mqtt

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;

const cluster = 0xFC00;
const attrs = {
    nfc_text:          0x0000,
    nfc_tag_uid:       0x0001,
    nfc_pending_write: 0x0002,
    nfc_reader_present: 0x0003,
};

// ── fromZigbee converters (device → coordinator) ──────────────────────

const fzNfcText = {
    cluster,
    type: ['readResponse', 'attributeReport'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data[attrs.nfc_text] !== undefined) {
            // ZCL char string: first byte is length
            const raw = msg.data[attrs.nfc_text];
            if (Buffer.isBuffer(raw)) {
                const len = raw[0];
                return { nfc_text: raw.slice(1, 1 + len).toString('utf8') };
            }
        }
    },
};

const fzNfcTagUid = {
    cluster,
    type: ['readResponse', 'attributeReport'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data[attrs.nfc_tag_uid] !== undefined) {
            const raw = msg.data[attrs.nfc_tag_uid];
            if (Buffer.isBuffer(raw)) {
                const len = raw[0];
                return { nfc_tag_uid: raw.slice(1, 1 + len).toString('hex') };
            }
        }
    },
};

const fzNfcPendingWrite = {
    cluster,
    type: ['readResponse', 'attributeReport'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data[attrs.nfc_pending_write] !== undefined) {
            const raw = msg.data[attrs.nfc_pending_write];
            if (Buffer.isBuffer(raw)) {
                const len = raw[0];
                return { nfc_pending_write: raw.slice(1, 1 + len).toString('utf8') };
            }
        }
    },
};

const fzNfcReaderPresent = {
    cluster,
    type: ['readResponse', 'attributeReport'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data[attrs.nfc_reader_present] !== undefined) {
            const val = msg.data[attrs.nfc_reader_present];
            return { nfc_reader_present: val === 1 };
        }
    },
};

// ── toZigbee converters (coordinator → device) ────────────────────────

const tzNfcPendingWrite = {
    key: ['nfc_pending_write'],
    convertSet: async (entity, key, value, meta) => {
        // ZCL char string: prepend length byte
        const buf = Buffer.alloc(1 + value.length);
        buf[0] = value.length;
        buf.write(value, 1, 'utf8');
        await entity.write(cluster, { [attrs.nfc_pending_write]: buf });
        return { state: { nfc_pending_write: value } };
    },
};

// ── Device definition ─────────────────────────────────────────────────

const definition = {
    // Must match the model string passed to setManufacturerAndModel()
    zigbeeModel: ['ZigbeeNFCEndpoint'],
    model: 'ZigbeeNFCBridge',
    vendor: 'Espressif',
    description: 'Zigbee NFC Reader/Writer Bridge (PN532)',
    fromZigbee: [fzNfcText, fzNfcTagUid, fzNfcPendingWrite, fzNfcReaderPresent],
    toZigbee: [tzNfcPendingWrite],
    exposes: [
        e.text('nfc_text', e.access.STATE)
            .withDescription('Last-read NFC tag text'),
        e.text('nfc_tag_uid', e.access.STATE)
            .withDescription('UID of the last-read NFC tag (hex)'),
        e.text('nfc_pending_write', e.access.ALL)
            .withDescription('Text queued for write to next presented tag'),
        e.binary('nfc_reader_present', e.access.STATE, true, false)
            .withDescription('Whether the PN532 module is reachable'),
    ],
    meta: { multiEndpoint: false },

    // Bind and configure reporting on interview
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        if (!endpoint) {
            logger.warn('NFC Bridge: endpoint 1 not found');
            return;
        }
        // Bind the custom cluster to the coordinator
        await reporting.bind(endpoint, coordinatorEndpoint, [cluster]);
        // Configure attribute reporting for nfc_reader_present
        await endpoint.configureReporting(cluster, [{
            attribute: { ID: attrs.nfc_reader_present, type: 0x10 },  // bool
            minimumReportInterval: 0,
            maximumReportInterval: 3600,  // 1 hour max
            reportableChange: 1,
        }]);
        logger.info('NFC Bridge: reporting configured');
    },
};

module.exports = definition;
