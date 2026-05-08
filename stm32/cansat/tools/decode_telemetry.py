#!/usr/bin/env python3
import argparse
import datetime as dt
import struct
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

TELEMETRY_START = 0xA5
TELEMETRY_TYPE_VERSION = 0x11
TELEMETRY_SIZE = 50


STATUS_BITS: Dict[int, str] = {
    0: "IMU_OK",
    1: "MAG_OK",
    2: "GPS_FIX",
    3: "INA_OK",
    4: "XBEE_DELIVERY_OK",
    5: "BME280_DISABLED",
    6: "GPS_FRESH",
    7: "IMU_FRESH",
    8: "INA_FRESH",
}


@dataclass
class Telemetry:
    sequence: int
    timestamp_ms: int
    gps_lat_e7: int
    gps_lon_e7: int
    gps_alt_cm: int
    gps_speed_cms: int
    gps_course_cdeg: int
    imu_ax_mg: int
    imu_ay_mg: int
    imu_az_mg: int
    imu_gx_dps10: int
    imu_gy_dps10: int
    imu_gz_dps10: int
    imu_mx_ut10: int
    imu_my_ut10: int
    imu_mz_ut10: int
    batt_mv: int
    batt_ma: int
    status_flags: int


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_telemetry(packet: bytes) -> Optional[Telemetry]:
    if len(packet) != TELEMETRY_SIZE:
        return None
    if packet[0] != TELEMETRY_START or packet[1] != TELEMETRY_TYPE_VERSION:
        return None

    expected_crc = struct.unpack_from("<H", packet, 48)[0]
    actual_crc = crc16_ccitt_false(packet[:48])
    if expected_crc != actual_crc:
        return None

    return Telemetry(
        sequence=struct.unpack_from("<H", packet, 2)[0],
        timestamp_ms=struct.unpack_from("<I", packet, 4)[0],
        gps_lat_e7=struct.unpack_from("<i", packet, 8)[0],
        gps_lon_e7=struct.unpack_from("<i", packet, 12)[0],
        gps_alt_cm=struct.unpack_from("<i", packet, 16)[0],
        gps_speed_cms=struct.unpack_from("<h", packet, 20)[0],
        gps_course_cdeg=struct.unpack_from("<h", packet, 22)[0],
        imu_ax_mg=struct.unpack_from("<h", packet, 24)[0],
        imu_ay_mg=struct.unpack_from("<h", packet, 26)[0],
        imu_az_mg=struct.unpack_from("<h", packet, 28)[0],
        imu_gx_dps10=struct.unpack_from("<h", packet, 30)[0],
        imu_gy_dps10=struct.unpack_from("<h", packet, 32)[0],
        imu_gz_dps10=struct.unpack_from("<h", packet, 34)[0],
        imu_mx_ut10=struct.unpack_from("<h", packet, 36)[0],
        imu_my_ut10=struct.unpack_from("<h", packet, 38)[0],
        imu_mz_ut10=struct.unpack_from("<h", packet, 40)[0],
        batt_mv=struct.unpack_from("<H", packet, 42)[0],
        batt_ma=struct.unpack_from("<h", packet, 44)[0],
        status_flags=struct.unpack_from("<H", packet, 46)[0],
    )


def seq_gap(previous: int, current: int) -> int:
    delta = (current - previous) & 0xFFFF
    if delta <= 1:
        return 0
    return delta - 1


def flags_to_names(flags: int) -> List[str]:
    out: List[str] = []
    for bit, name in STATUS_BITS.items():
        if flags & (1 << bit):
            out.append(name)
    return out


class XBeeApiStreamParser:
    def __init__(self, escaped: bool) -> None:
        self.escaped = escaped
        self.state = "wait_start"
        self.escape_next = False
        self.length = 0
        self.frame = bytearray()

    def feed(self, b: int) -> Iterable[bytes]:
        if self.state == "wait_start":
            if b == 0x7E:
                self.state = "len_msb"
                self.escape_next = False
                self.length = 0
                self.frame.clear()
            return []

        if self.escaped:
            if self.escape_next:
                b ^= 0x20
                self.escape_next = False
            elif b == 0x7D:
                self.escape_next = True
                return []

        if self.state == "len_msb":
            self.length = b << 8
            self.state = "len_lsb"
            return []
        if self.state == "len_lsb":
            self.length |= b
            self.state = "data"
            self.frame.clear()
            return []
        if self.state == "data":
            self.frame.append(b)
            if len(self.frame) >= self.length:
                self.state = "checksum"
            return []
        if self.state == "checksum":
            checksum_ok = ((sum(self.frame) + b) & 0xFF) == 0xFF
            payload = bytes(self.frame) if checksum_ok else b""
            self.state = "wait_start"
            self.frame.clear()
            return [payload] if payload else []

        self.state = "wait_start"
        return []


def get_rf_payload(frame_data: bytes) -> Optional[bytes]:
    if not frame_data:
        return None

    frame_type = frame_data[0]
    if frame_type == 0x90 and len(frame_data) >= 12:
        return frame_data[12:]
    if frame_type == 0x91 and len(frame_data) >= 18:
        return frame_data[18:]
    if frame_type == 0x10 and len(frame_data) >= 14:
        return frame_data[14:]
    return None


def find_telemetry_packets(payload: bytes) -> Iterable[bytes]:
    if len(payload) < TELEMETRY_SIZE:
        return []
    out: List[bytes] = []
    for i in range(0, len(payload) - TELEMETRY_SIZE + 1):
        if payload[i] == TELEMETRY_START and payload[i + 1] == TELEMETRY_TYPE_VERSION:
            out.append(payload[i : i + TELEMETRY_SIZE])
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Decode CanSat telemetry from XBee serial stream")
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbserial-XXXX")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    p.add_argument(
        "--escaped",
        action="store_true",
        help="Enable XBee AP=2 unescape processing (AP=1 by default)",
    )
    return p.parse_args()


def main() -> int:
    try:
        import serial  # type: ignore
    except ImportError as exc:  # pragma: no cover
        print("pyserial is required. Install with: python3 -m pip install pyserial", file=sys.stderr)
        raise SystemExit(1) from exc

    args = parse_args()
    parser = XBeeApiStreamParser(escaped=args.escaped)
    last_seq: Optional[int] = None
    total_lost = 0

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(
            "ts,seq,lost,total_lost,time_s,lat,lon,alt_m,speed_mps,course_deg,"
            "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,batt_v,batt_a,flags_hex,flags"
        )
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            for raw in chunk:
                for frame in parser.feed(raw):
                    rf_payload = get_rf_payload(frame)
                    if rf_payload is None:
                        continue
                    for candidate in find_telemetry_packets(rf_payload):
                        t = decode_telemetry(candidate)
                        if t is None:
                            continue

                        lost = 0
                        if last_seq is not None:
                            lost = seq_gap(last_seq, t.sequence)
                            total_lost += lost
                        last_seq = t.sequence

                        now = dt.datetime.now().strftime("%H:%M:%S")
                        flag_names = "|".join(flags_to_names(t.status_flags))
                        print(
                            f"{now},{t.sequence},{lost},{total_lost},{t.timestamp_ms / 1000:.3f},"
                            f"{t.gps_lat_e7 / 1e7:.7f},{t.gps_lon_e7 / 1e7:.7f},{t.gps_alt_cm / 100:.2f},"
                            f"{t.gps_speed_cms / 100:.2f},{t.gps_course_cdeg / 100:.2f},"
                            f"{t.imu_ax_mg / 1000:.3f},{t.imu_ay_mg / 1000:.3f},{t.imu_az_mg / 1000:.3f},"
                            f"{t.imu_gx_dps10 / 10:.1f},{t.imu_gy_dps10 / 10:.1f},{t.imu_gz_dps10 / 10:.1f},"
                            f"{t.imu_mx_ut10 / 10:.1f},{t.imu_my_ut10 / 10:.1f},{t.imu_mz_ut10 / 10:.1f},"
                            f"{t.batt_mv / 1000:.3f},{t.batt_ma / 1000:.3f},"
                            f"0x{t.status_flags:04X},{flag_names}",
                            flush=True,
                        )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
