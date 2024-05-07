#!/usr/bin/env python3

import sys
import serial

import time

MEMORY_SIZE = 65536

def main(*argv) -> None:
    if len(argv) == 2:
        file = argv[1]
        ser_port = '/dev/ttyACM0'
    elif len(argv) == 3:
        file = argv[2]
        ser_port = argv[1]
    else:
        print("Usage: {} [serial port] file.bin".format(argv[0]), file=sys.stderr)
        return
    ser = serial.Serial(ser_port, 921600, timeout=0.1)

    bf = ""
    with open(file, 'rb') as hfile:
        bf = hfile.read()
        if len(bf) != MEMORY_SIZE:
            raise ValueError(f"Input file size does not match storage size: given {len(bf)} bytes, wanted {MEMORY_SIZE} bytes")

    for i in range(0, MEMORY_SIZE, 4):
        data = bf[i]
        data += bf[i+1] << 8
        data += bf[i+2] << 16
        data += bf[i+3] << 24

        if data == 0:
            continue

        ser.write(f"{data:08x} {i:08x}\r\n".encode())
        chk = data ^ i
        res = ser.read(8 + 2)
        #print(f"Expected: {chk:08x}, Got: {res.decode()[:-2]}")
        assert (res == (f"{chk:08x}\r\n").encode())

    end_val = (1 << 32) - 1
    ser.write(f"{end_val:08x} {end_val:08x}".encode())
    print("Programming Done!", file=sys.stderr)
    ser.close()


if __name__ == "__main__":
    main(*sys.argv)
