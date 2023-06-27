import time
import serial
import json

device = serial.Serial("/dev/ttyUSB0", 9600)

state = False
buffer = b""


def write(data: dict) -> None:
    device.write(json.dumps(data).encode() + b"\n")


try:
    while True:
        data = {"command": "laser", "state": state}
        state = not state
        write(data)
        if state:
            write({"command": "servo", "channel": 1, "angle": 45})
            write({"command": "servo", "channel": 2, "angle": 45})
        else:
            write({"command": "servo", "channel": 1, "angle": 135})
            write({"command": "servo", "channel": 2, "angle": 135})
        time.sleep(0.5)
        if device.in_waiting > 0:
            buffer += device.read(device.in_waiting)
            if buffer[-1] != ord("\n"):
                index = buffer.rfind(b"\n")
                packets = buffer[:index]
                buffer = buffer[index + 1 :]
            else:
                packets = buffer
            for line in packets.splitlines():
                try:
                    data = json.loads(line.decode())
                except (json.decoder.JSONDecodeError, UnicodeDecodeError):
                    pass
                print(data)
finally:
    device.close()
