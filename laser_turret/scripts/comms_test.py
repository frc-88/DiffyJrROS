import time
import serial

device = serial.Serial("/dev/ttyUSB0", 9600)


try:
    while True:
        device.write(b"t")
        time.sleep(0.75)
        device.write(b"t")
        time.sleep(0.75)
        if device.in_waiting > 0:
            print(device.read(device.in_waiting))
finally:
    device.close()
