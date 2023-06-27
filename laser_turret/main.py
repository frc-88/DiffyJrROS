from machine import Pin, PWM, UART
import utime
import ujson


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


def main():
    def data_callback(data: dict) -> None:
        if type(data) != dict:
            return
        command = data.get("command", None)
        if command == "laser":
            state = bool(data.get("state", False))
            laser.set(state)
        elif command == "servo":
            servo_num = data.get("channel", 0)
            angle = data.get("angle", 0)
            if servo_num == 1:
                servo1.set(angle)
            elif servo_num == 2:
                servo2.set(angle)

    def publish_state() -> None:
        data = {
            "servo1": servo1.current_angle,
            "servo2": servo2.current_angle,
            "laser": laser.led.value(),
        }
        comms.write(ujson.dumps(data).encode() + b"\n")

    servo1 = Servo(2)
    servo2 = Servo(3)
    laser = Laser(6)
    comms = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
    laser.set(True)
    buffer = b""
    prev_report_time = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()
        if now - prev_report_time > 100:
            publish_state()
            prev_report_time = now
        if comms.any():
            char = comms.read(1)
            if char == b"\n":
                try:
                    data = ujson.loads(buffer.decode())
                    data_callback(data)
                    buffer = b""
                except (ujson.JSONDecodeError, UnicodeDecodeError):
                    pass
            else:
                buffer += char


if __name__ == "__main__":
    main()
