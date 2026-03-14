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
    pan_vel = 0
    tilt_vel = 0

    def tx(cmd):
        ser.write((cmd + "\n").encode())
        print(f"-> {cmd}")

    def tx_pair():
        tx(f"{pan_vel:+d},T{tilt_vel:+d}")

    def on_press(key):
        nonlocal pan_vel, tilt_vel
        if key in held: return
        held.add(key)
        if   key == keyboard.Key.right: pan_vel = +PAN_VEL
        elif key == keyboard.Key.left:  pan_vel = -PAN_VEL
        elif key == keyboard.Key.up:    tilt_vel = +TILT_VEL
        elif key == keyboard.Key.down:  tilt_vel = -TILT_VEL
        elif hasattr(key, "char") and key.char:
            c = key.char
            if   c == "q": return False
            elif c == "s":
                pan_vel = 0
                tilt_vel = 0
                tx("S")
                return
        tx_pair()

    def on_release(key):
        nonlocal pan_vel, tilt_vel
        held.discard(key)
        if   key in (keyboard.Key.right, keyboard.Key.left):  pan_vel = 0
        elif key in (keyboard.Key.up, keyboard.Key.down):     tilt_vel = 0
        else: return
        tx_pair()

    print("Arrows=pan/tilt  s=stop  q=quit")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as l:
        l.join()

    tx("S"); ser.close()

if __name__ == "__main__":
    main()