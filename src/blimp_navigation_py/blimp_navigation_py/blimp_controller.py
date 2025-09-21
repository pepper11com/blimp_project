#!/usr/bin/env python3
import time, struct, threading, math
from yamspy import MSPy

# --- MSP Constants and Helpers ---
MSP2_INAV_SET_SERVO_OVERRIDE = 0x20F6
MSP2_INAV_SET_MOTOR_OVERRIDE = 0x20F5
UPDATE_HZ = 20

def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def build_mspv2_native(cmd: int, payload: bytes, flags: int = 0) -> bytes:
    hdr = struct.pack('<3sBHH', b'$X<', flags & 0xFF, cmd & 0xFFFF, len(payload) & 0xFFFF)
    crc = crc8_dvb_s2(hdr[3:] + payload)
    return hdr + payload + bytes([crc])

def pct_to_us(pct: float) -> int:
    pct = max(0.0, min(100.0, float(pct)))
    return int(1000 + pct * 10)

class BlimpController:
    def __init__(self, device='/dev/ttyAMA0', baud=115200, servo_payload_len=4, motor_count=2):
        self.board = MSPy(device=device, baudrate=baud, loglevel='WARNING')
        if self.board.connect() != 0:
            raise RuntimeError("MSP connection failed")
        self._lock = threading.Lock()
        self.servo_values = [1500] * servo_payload_len
        self.motor_values = [1000] * motor_count
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self._run = True
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._attitude_thread = threading.Thread(target=self._attitude_loop, daemon=True)
        self._sender_thread.start()
        self._attitude_thread.start()

    def set_servo_us(self, idx, us):
        us = max(500, min(2500, int(us)))
        value_to_set = us
        if idx == 2:
            value_to_set = 3000 - us
        with self._lock:
            if 0 <= idx < len(self.servo_values):
                self.servo_values[idx] = value_to_set

    def set_motor_pct(self, motor_idx, pct):
        us = pct_to_us(pct)
        with self._lock:
            if 0 <= motor_idx < len(self.motor_values):
                self.motor_values[motor_idx] = us

    def _send_servos(self):
        with self._lock:
            payload = struct.pack('<' + 'H' * len(self.servo_values), *self.servo_values)
        frame = build_mspv2_native(MSP2_INAV_SET_SERVO_OVERRIDE, payload)
        self.board.conn.write(frame)

    def _send_motors(self):
        with self._lock:
            payload = struct.pack('<' + 'H' * len(self.motor_values), *self.motor_values)
        frame = build_mspv2_native(MSP2_INAV_SET_MOTOR_OVERRIDE, payload)
        self.board.conn.write(frame)

    def _sender_loop(self):
        dt = 1.0 / UPDATE_HZ
        while self._run:
            try:
                self._send_servos()
                self._send_motors()
                self.board.conn.flush()
            except Exception as e:
                print(f"BlimpController Send error: {e}")
            time.sleep(dt)

    def _build_msp_frame(self, cmd, payload=b''):
        size = len(payload)
        frame = b'$M<' + bytes([size, cmd]) + payload
        checksum = size ^ cmd
        for b in payload: checksum ^= b
        return frame + bytes([checksum & 0xFF])

    def _parse_attitude_response(self, response):
        try:
            frame_start = response.find(b'$M>')
            if frame_start != -1 and len(response) > frame_start + 4:
                size = response[frame_start + 3]
                cmd = response[frame_start + 4]
                if cmd == 108 and size >= 6:
                    payload = response[frame_start + 5 : frame_start + 5 + size]
                    roll_raw, pitch_raw, heading_raw = struct.unpack('<3h', payload[:6])
                    with self._lock:
                        self.roll = roll_raw / 10.0
                        self.pitch = pitch_raw / 10.0
                        self.heading = heading_raw
                        if self.heading < 0: self.heading += 360
                    return True
        except Exception as e:
            print(f"BlimpController Attitude parse error: {e}")
        return False

    def _attitude_loop(self):
        while self._run:
            try:
                ser = self.board.conn
                ser.reset_input_buffer()
                frame = self._build_msp_frame(108) # MSP_ATTITUDE
                ser.write(frame)
                ser.flush()
                time.sleep(0.05)
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    self._parse_attitude_response(response)
            except Exception as e:
                print(f"BlimpController Attitude read error: {e}")
            time.sleep(0.1)

    def stop(self):
        self._run = False
        time.sleep(0.2)
        try:
            self.set_motor_pct(0, 0); self.set_motor_pct(1, 0)
            self.set_servo_us(2, 1500); self.set_servo_us(3, 1500)
            time.sleep(0.1)
        except: pass
        try:
            if hasattr(self.board, 'close'):
                self.board.close()
            elif hasattr(self.board, 'conn') and hasattr(self.board.conn, 'close'):
                self.board.conn.close()
        except: pass