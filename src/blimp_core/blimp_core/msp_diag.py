#!/usr/bin/env python3
import os, sys, time, argparse, binascii
from contextlib import contextmanager
try:
    import serial
except ImportError:
    print("pip install pyserial"); sys.exit(1)

def hx(b, n=128): s=' '.join(f'{x:02X}' for x in b[:n]); return s+ (f' ...(+{len(b)-n})' if len(b)>n else '')
def msp_ident(): return b'\x24\x4D\x3C\x00\x64\x64'  # $ M < size=0 cmd=0x64 chk=0x64
def parse_v1(resp):
    if len(resp)<6: return (False,"Too short")
    try: i=resp.index(b'\x24')
    except ValueError: return (False,"No '$'")
    if resp[i:i+2]!=b'\x24\x4D': return (False,"Bad sig")
    if resp[i+2] not in (0x3E,0x21): return (False,"No response header")
    size=resp[i+3]; cmd=resp[i+4]; pl=resp[i+5:i+5+size]; chk=resp[i+5+size]
    calc=size ^ cmd
    for b in pl: calc^=b
    return (calc==chk, f"cmd=0x{cmd:02X} size={size} chk_ok={calc==chk} payload={hx(pl)}")

def try_port(port, baud=115200):
    print(f"\n[PORT] {port} @{baud}")
    try:
        ser=serial.Serial(port, baud, timeout=0.4)
    except Exception as e:
        print("  Open failed:", e); return False
    time.sleep(0.1); ser.reset_input_buffer()
    time.sleep(0.2); passive=ser.read(64)
    print("  Passive:", len(passive), "bytes:", hx(passive) if passive else "(none)")
    ok=False
    for i in range(3):
        ser.write(msp_ident()); ser.flush(); time.sleep(0.15)
        r=ser.read(128); print(f"  Try {i+1}: {len(r)} bytes:", hx(r))
        good, info = parse_v1(r); print("  Parse:", info)
        if good: ok=True; break
    try:
        from yamspy import MSPy
        print("  YAMSPy handshake...")
        b=MSPy(device=port, baudrate=baud, loglevel='WARNING')
        if b.connect():
            print("  Connected via YAMSPy")
            att=b.getData(MSPy.MSPCodes['MSP_ATTITUDE'])
            print("  MSP_ATTITUDE:", att)
            b.disconnect()
        else:
            print("  YAMSPy connect() failed")
    except ImportError:
        print("  (Install yamspy for high-level check)")
    except Exception as e:
        print("  YAMSPy error:", e)
    ser.close()
    return ok

if __name__=="__main__":
    ports=["/dev/ttyAMA0","/dev/ttyAMA10","/dev/ttyS0","/dev/ttyACM0"]
    any_ok=False
    for p in ports:
        if os.path.exists(p):
            any_ok |= try_port(p)
    print("\nRESULT:", "MSP responding on at least one port" if any_ok else "No MSP response; check UART selection/wiring/Ports tab")
    sys.exit(0 if any_ok else 2)
