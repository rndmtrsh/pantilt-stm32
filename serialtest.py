#!/usr/bin/env python3
import sys, serial
from pynput import keyboard

PAN_VEL = 1000
TILT_VEL = 500

def find_port():
    for i in range(3):
        try:
            s = serial.Serial(f"/dev/ttyACM{i}")
            s.close()
            return f"/dev/ttyACM{i}"
        except (OSError, serial.SerialException):
            pass
    return None

def main():
    port = find_port()
    if not port:
        print("STM32 tidak ditemukan."); sys.exit(1)

    ser = serial.Serial(port, 115200, timeout=1)
    print(f"Terhubung ke {port}")
    held = set()

    def tx(cmd):
        ser.write((cmd + "\n").encode())
        print(f"-> {cmd}")

    def on_press(key):
        if key in held: return
        held.add(key)
        if   key == keyboard.Key.right: tx(f"P+{PAN_VEL}")
        elif key == keyboard.Key.left:  tx(f"P-{PAN_VEL}")
        elif key == keyboard.Key.up:    tx(f"T+{TILT_VEL}")
        elif key == keyboard.Key.down:  tx(f"T-{TILT_VEL}")
        elif hasattr(key, "char") and key.char:
            c = key.char
            if   c == "q": return False
            elif c == "s": tx("S")
            elif c == "?": tx("?")
            elif c == "1": tx("IP")
            elif c == "2": tx("IT")

    def on_release(key):
        held.discard(key)
        if   key in (keyboard.Key.right, keyboard.Key.left):  tx("P+0")
        elif key in (keyboard.Key.up, keyboard.Key.down):     tx("T+0")

    print("Arrows=pan/tilt  s=stop  1/2=invert  ?=status  q=quit")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as l:
        l.join()

    tx("S"); ser.close()

if __name__ == "__main__":
    main()