from machine import Pin, PWM, UART
import utime
import json


class Servo:
    _servo_pwm_freq = 50
    _min_u16_duty = 1640 - 2  # offset for correction
    _max_u16_duty = 7864 - 0  # offset for correction
    min_angle = 0
    max_angle = 180
    current_angle = 0.001

    def __init__(self, pin):
        self._initialize(pin)

    def update_settings(
        self, servo_pwm_freq, min_u16_duty, max_u16_duty, min_angle, max_angle, pin
    ):
        self._servo_pwm_freq = servo_pwm_freq
        self._min_u16_duty = min_u16_duty
        self._max_u16_duty = max_u16_duty
        self.min_angle = min_angle
        self.max_angle = max_angle
        self._initialize(pin)

    def set(self, angle):
        # round to 2 decimal places, so we have a chance of reducing unwanted servo adjustments
        angle = round(angle, 2)
        # do we need to move?
        if angle == self.current_angle:
            return
        self.current_angle = angle
        # calculate the new duty cycle and move the motor
        duty_u16 = self._angle_to_u16_duty(angle)
        self._motor.duty_u16(duty_u16)

    def _angle_to_u16_duty(self, angle):
        return (
            int((angle - self.min_angle) * self._angle_conversion_factor)
            + self._min_u16_duty
        )

    def _initialize(self, pin):
        self.current_angle = -0.001
        self._angle_conversion_factor = (self._max_u16_duty - self._min_u16_duty) / (
            self.max_angle - self.min_angle
        )
        self._motor = PWM(Pin(pin))
        self._motor.freq(self._servo_pwm_freq)


class Laser:
    def __init__(self, pin_num: int) -> None:
        self.led = Pin("LED", Pin.OUT)
        self.laser = Pin(pin_num, Pin.OUT)

    def set(self, state: bool) -> None:
        self.led.value(state)
        self.laser.value(state)

    def toggle(self) -> None:
        self.led.toggle()
        self.laser.toggle()


class Comms:
    def __init__(self, tx_pin: int, rx_pin: int) -> None:
        self.uart = UART(1, baudrate=9600, tx=Pin(tx_pin), rx=Pin(rx_pin))

    def write(self, data: str) -> None:
        self.uart.write(data)


def main():
    servo1 = Servo(2)
    servo2 = Servo(3)
    laser = Laser(6)
    comms = Comms(8, 9)
    laser.set(True)

    start_time = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()
        if now - start_time > 1000:
            comms.uart.write("something\r\n")
            start_time = now
        if comms.uart.any():
            char = comms.uart.read(1)
            laser.toggle()
            if char == "e":
                comms.uart.write("something else\n")
            elif char == "a":
                servo1.set(45)
                servo2.set(45)
            elif char == "b":
                servo1.set(135)
                servo2.set(135)


if __name__ == "__main__":
    main()
