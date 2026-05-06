import * as fs from 'fs/promises'
import * as path from 'path';
import { setTimeout } from 'node:timers/promises';
import { SerialPort } from 'serialport';

const PACKET_LENGTH_BYTES = 1;
const PACKET_DATA_BYTES = 16;
const PACKET_CRC_BYTES = 1;
const PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
const PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;

const PACKET_ACK_DATA0 = 0x15;
const PACKET_RETX_DATA0 = 0x19;

const BL_PACKET_SYNC_OBSERVED_DATA0 =  (0X20);
const BL_PACKET_FW_UPDATE_REQ_DATA0 =  (0X31);
const BL_PACKET_FW_UPDATE_RES_DATA0 =  (0x37);
const BL_PACKET_DEVICE_ID_REQ_DATA0 =  (0x3C);
const BL_PACKET_DEVICE_ID_RES_DATA0 =  (0x3F);
const BL_PACKET_FW_LENGTH_REQ_DATA0 =  (0x42);
const BL_PACKET_FW_LENGTH_RES_DATA0 =  (0x45);
const BL_PACKET_READY_FOR_DATA_DATA0 = (0x48);
const BL_PACKET_UPDATE_SUCCESS_DATA0 = (0x54);
const BL_PACKET_NACK_DATA0 = (0x59);

const FLASH_BASE                        = (0x8000000);
const BOOTLOADER_SIZE                   = (0x8000);
const VECTOR_TABLE_SIZE                 = (0xE0);
const FIRMWARE_INFO_SIZE        = (10 * 4);

const FW_INFO_SENTINEL                  = (0xDEADC0DE);
const FW_INFO_ADDRESS           = (FLASH_BASE + BOOTLOADER_SIZE + VECTOR_TABLE_SIZE);
const FW_INFO_VALIDATE_FROM     = (FW_INFO_ADDRESS + FIRMWARE_INFO_SIZE);

const FW_INFO_DEVICE_ID_OFFSET  = (FW_INFO_ADDRESS + (1 * 4));
const FW_INFO_VERSION_OFFSET    = (FW_INFO_ADDRESS + (2 * 4));
const FW_INFO_LENGTH_OFFSET     = (FW_INFO_ADDRESS + (3 * 4));
const FW_INFO_CRC_OFFSET        = (FW_INFO_ADDRESS + (9 * 4));

const DEVICE_ID = (0x42);
const SYNC_SEQ = Buffer.from([0xC4, 0x55, 0x7E, 0x10]);
const BOOTLOADER_TIMEOUT_MS = (5000);

const serialPath = "/dev/ttyACM0"; // Update this to your serial port path
const baudRate = 115200;

const crc8 = (data: Buffer | Array<number>) => {
    let crc = 0;

    for (const byte of data) {
        crc = (crc ^ byte) & 0xFF;
        for (let i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xFF;
            } else {
                crc = (crc << 1) & 0xFF;
            }
        }
    }

    return crc;
};

const crc32 = (data: Buffer, length: number) => {
    let byte;
    let crc = 0xFFFFFFFF;
    let mask;

    for (let i = 0; i < length; i++) {
        byte = data[i];
        crc = (crc ^ byte) >>> 0;

        for (let j = 0; j < 8; j++) {
            mask = (-(crc & 1)) >>> 0;
            crc = ((crc >>> 1) ^ (0xEDB88320 & mask)) >>> 0;
        }
    }

    return (~crc) >>> 0;
}

const delay = (ms: number) => setTimeout(ms);

class Logger {
    static info(message: string) {
        console.log(`[.INFO] ${message}`);
    }
    static error(message: string) {
        console.error(`[!ERROR] ${message}`);
    }
    static success(message: string) {
        console.log(`[$SUCCESS] ${message}`);
    }
}

class Packet {
    length: number;
    data: Buffer;
    crc: number;

    static ack = new Packet(1, Buffer.from([PACKET_ACK_DATA0])).toBuffer();
    static retx = new Packet(1, Buffer.from([PACKET_RETX_DATA0])).toBuffer();

    constructor(length: number, data: Buffer, crc?: number) {
        this.length = length;
        this.data = data;

        if (this.data.length < PACKET_DATA_BYTES) {
            const bytesToPad = PACKET_DATA_BYTES - this.length;
            const padding = Buffer.alloc(bytesToPad, 0xFF);
            this.data = Buffer.concat([this.data, padding]);
        }

        if (typeof crc === 'undefined') {
            this.crc = this.computeCrc();
        } else {
            this.crc = crc;
        }
    }

    computeCrc() {
        const allData = [this.length, ...this.data];
        return crc8(allData);
    }

    toBuffer() {
        return Buffer.concat([
            Buffer.from([this.length]), this.data, Buffer.from([this.crc])]);
    }

    isSingleBytePacket(byte: number) {
        if (this.length !== 1) return false;
        if (this.data[0] !== byte) return false;
        for (let i = 1; i < PACKET_DATA_BYTES; i++) {
            if (this.data[i] !== 0xFF) return false;
        }
        return true;
    }

    isAck() {
        return this.isSingleBytePacket(PACKET_ACK_DATA0);
    }

    isRetx() {
        return this.isSingleBytePacket(PACKET_RETX_DATA0);
    }

    static createSingleBytePacket(byte: number) {
        return new Packet(1, Buffer.from([byte]));
    }
}

// Serial port instance
const uart = new SerialPort({ path: serialPath, baudRate });

// Packet buffer
let packets: Packet[] = [];

let lastPacket: Buffer = Packet.ack

const writePacket = (packet: Buffer) => {
    uart.write(packet);
    lastPacket = packet;
}

let rxBuffer = Buffer.from([]);
const consumeFromBuffer = (n: number) => {
    const consumed = rxBuffer.slice(0, n);
    rxBuffer = rxBuffer.slice(n);
    return consumed;
}

uart.on('data', data => {
    // Add data to the packet
    rxBuffer = Buffer.concat([rxBuffer, data]);

    // Check if we can build a packet
    while (rxBuffer.length >= PACKET_LENGTH) {
        const raw = consumeFromBuffer(PACKET_LENGTH);
        const packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        const computedCrc = packet.computeCrc();

        // Check if the packet is valid and request a retransmission if not
        if (computedCrc !== packet.crc) {
            console.log(`CRC failed, computed 0x${computedCrc.toString(16)}, got 0x${packet.crc.toString(16)}`)
            writePacket(Packet.retx);
            continue;
        }

        if (packet.isRetx()) {
            console.log(`Retransmitting last packet`);
            writePacket(lastPacket);
            continue;
        }

        if (packet.isAck()) {
            continue;
        }

        if(packet.isSingleBytePacket(BL_PACKET_NACK_DATA0)) {
            Logger.error("Received NACK packet, exiting...")
            process.exit(1);
        }

        packets.push(packet);
        writePacket(Packet.ack);
    }
});

// Wait for a packet to be available
const waitForPacket = async (timeout = BOOTLOADER_TIMEOUT_MS) => {
    let timeWaited = 0;
    while (packets.length < 1) {
        await delay(1);
        timeWaited += 1;

        if (timeWaited >= timeout) {
            Logger.error("Timeout waiting for packet");
            throw Error("Timeout waiting for packet");
        }
    }
    const packet = packets[0];
    packets = packets.slice(1);
    return packet;
}

const waitForSingleBytePacket = (byte: number, timeout = BOOTLOADER_TIMEOUT_MS) => (
    waitForPacket(timeout)
        .then(packet => {
            if (!packet.isSingleBytePacket(byte)) {
                throw new Error(`Expected packet with data0=0x${byte.toString(16)}, got 0x${packet.data[0].toString(16)}`);
            }
        })
        .catch((e: Error) => {
            Logger.error(e.message);
            process.exit(1);
        })
)

const syncWithBootloader = async (timeout = BOOTLOADER_TIMEOUT_MS) => {
    let timeWaited = 0;
    while (true) {
        uart.write(SYNC_SEQ);
        await delay(1000);
        timeWaited += 1000;

        if (packets.length > 0) {
            const packet = packets.splice(0, 1)[0];
            if (packet.isSingleBytePacket(BL_PACKET_SYNC_OBSERVED_DATA0)) {
                Logger.info("Bootloader sync observed");
                return;
            }
            Logger.error(`Expected bootloader sync, got packet with data0=0x${packet.data[0].toString(16)}`);
            process.exit(1);
        }

        if (timeWaited >= timeout) {
            Logger.error("Timeout waiting for sync sequence");
            process.exit(1);
        }
    }
}

const main = async () => {
    Logger.info("Reading firmware file...");
    const fwImage = await fs.readFile(path.join(process.cwd(), "dev_tools/baremetal_app.bin"))
        .then(bin => bin.slice(BOOTLOADER_SIZE));
    const fwLength = fwImage.length;
    Logger.success(`Read firmware file, length ${fwLength} bytes`);

    Logger.info("Injecting into firmware information section");
    fwImage.writeUInt32LE(0x00000001, FW_INFO_VERSION_OFFSET);
    fwImage.writeUInt32LE(fwLength, FW_INFO_LENGTH_OFFSET);

    const crcValue = crc32(fwImage.slice(FW_INFO_VALIDATE_FROM), fwLength - (VECTOR_TABLE_SIZE + FIRMWARE_INFO_SIZE));
    Logger.info(`Calculated CRC32 of firmware: 0x${crcValue.toString(16).padStart(8, '0')}`);
    fwImage.writeUInt32LE(crcValue, FW_INFO_CRC_OFFSET);


    Logger.info("Attempting to sync with bootloader...");
    await syncWithBootloader();
    Logger.success("Synced with bootloader");

    const fwUpdatePacket = Packet.createSingleBytePacket(BL_PACKET_FW_UPDATE_REQ_DATA0).toBuffer();
    writePacket(fwUpdatePacket);
    Logger.info("Sent firmware update request");

    await waitForSingleBytePacket(BL_PACKET_FW_UPDATE_RES_DATA0);
    Logger.success("Firmware update request accepted");

    Logger.info("Waiting for device ID request...");
    await waitForSingleBytePacket(BL_PACKET_DEVICE_ID_REQ_DATA0);

    const deviceIdPacket = new Packet(2, Buffer.from([BL_PACKET_DEVICE_ID_RES_DATA0, DEVICE_ID])).toBuffer();
    writePacket(deviceIdPacket);
    Logger.info("Responded with device ID 0x" + DEVICE_ID.toString(16));

    Logger.info("Waiting for firmware length request...");
    await waitForSingleBytePacket(BL_PACKET_FW_LENGTH_REQ_DATA0);

    const fwLengthPacketBuffer = Buffer.alloc(5);
    fwLengthPacketBuffer[0] = BL_PACKET_FW_LENGTH_RES_DATA0;
    fwLengthPacketBuffer.writeUInt32LE(fwLength, 1);
    const fwLengthPacket = new Packet(5, fwLengthPacketBuffer).toBuffer();
    writePacket(fwLengthPacket);
    Logger.info("Responded with firmware length");

    Logger.info("Waiting for a few seconds for main application to be erased...")
    await delay(3000);

    let bytesWritten = 0;
    while (bytesWritten < fwLength) {
        await waitForSingleBytePacket(BL_PACKET_READY_FOR_DATA_DATA0, 90000);
        const chunk = fwImage.slice(bytesWritten, bytesWritten + PACKET_DATA_BYTES);
        const chunkLength = chunk.length;

        const chunkPacket = new Packet(chunkLength - 1, chunk).toBuffer();
        writePacket(chunkPacket);
        bytesWritten += chunkLength;
        Logger.info(`Wrote ${chunkLength} bytes of firmware (${bytesWritten}/${fwLength})`);
    }
    Logger.info("All firmware bytes written. Waiting for update success from bootloader...");

    await waitForSingleBytePacket(BL_PACKET_UPDATE_SUCCESS_DATA0);
    Logger.success("Firmware update complete!");
}

main().finally(() => uart.close());
