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
    console.log(`Received ${data.length} bytes through UART`);
    // Add data to the packet
    rxBuffer = Buffer.concat([rxBuffer, data]);

    // Check if we can build a packet
    while (rxBuffer.length >= PACKET_LENGTH) {
        console.log(`Building packet from ${rxBuffer.length} bytes`);
        const raw = consumeFromBuffer(PACKET_LENGTH);
        const packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        const computedCrc = packet.computeCrc();

        // Check if the packet is valid and request a retransmission if not
        if (computedCrc !== packet.crc) {
            console.log(`CRC failed, computed 0x${computedCrc.toString(16)}, got 0x${packet.crc.toString(16)}`)
            writePacket(Packet.retx);
            return;
        }

        if (packet.isRetx()) {
            console.log(`Retransmitting last packet`);
            writePacket(lastPacket);
            return;
        }

        if (packet.isAck()) {
            console.log(`ACK received`)
            return;
        }

        console.log(`Storing packet and ack'ing`);
        packets.push(packet);
        console.log(`ACK sent`)
        writePacket(Packet.ack);
    }
});

// Wait for a packet to be available
const waitForPacket = async () => {
    while (packets.length < 1) {
        await delay(1);
    }
    const packet = packets[0];
    packets = packets.slice(1);
    return packet;
}

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
        }
    }
}

const main = async () => {
    Logger.info("Attempting to sync with bootloader...");
    await syncWithBootloader();
    Logger.success("Synced with bootloader");
}

main();
