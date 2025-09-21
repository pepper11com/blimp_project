msp_diag.py

A small diagnostic script to probe INAV/Betaflight MSP over a serial UART.

Usage

Run directly (needs `pyserial`):

```bash
python3 msp_diag.py --ports /dev/ttyS0 /dev/ttyAMA0 --baud 115200
```

Or make executable and run:

```bash
chmod +x msp_diag.py
./msp_diag.py
```

Notes

- The script tries passive reads and sends MSP IDENT requests (MSP v1) to detect a responding flight controller.
- If `yamspy` is installed, the script will attempt a high-level connection.
- Install dependencies: `pip install pyserial yamspy` (yamspy optional).