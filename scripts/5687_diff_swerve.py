#!/usr/bin/env python3

import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as fct
import control as ct

import math
import matplotlib.pyplot as plt
import numpy as np


class Constants:
    VOLTAGE = 12.0
    J_w = 0.005
    J_a = 0.004
    
    GEAR_RATIO_WHEEL = 6.46875
    GEAR_RATIO_STEER = 9.2

    Q_AZIMUTH = 1.5  # radians
    Q_WHEEL_ANG_VELOCITY = 5.0  # radians per sec
    Q_AZIMUTH_ANG_VELOCITY = 0.8  # radians per sec

    MODEL_AZIMUTH_ANGLE_NOISE = 0.1  # radians
    MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0  # radians per sec
    MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0  # radians per sec

    SENSOR_AZIMUTH_ANGLE_NOISE = 0.01  # radians
    SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1  # radians per sec
    SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1  # radians per sec


class DiffSwerve(fct.System):
    def __init__(self, dt):
        """Diff Swerve subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Azimuth", "rad"), ("Azimuth velocity", "rad/s"), ("Wheel velocity", "rad/s")]
        u_labels = [("Left Voltage", "V"), ("Right Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([
                [-Constants.VOLTAGE],
                [-Constants.VOLTAGE]]
            ),
            np.array([
                [Constants.VOLTAGE],
                [Constants.VOLTAGE]]
            ),
            dt,
            np.zeros((3, 1)),
            np.zeros((2, 1)),
        )

    def create_model(self, states, inputs):
        motor = fct.models.MOTOR_FALCON_500
        Gs = Constants.GEAR_RATIO_STEER
        Gw = Constants.GEAR_RATIO_WHEEL
        J_a = Constants.J_a
        J_w = Constants.J_w
        Cs = -((Gs * motor.Kt) / (motor.Kv * motor.R * J_a))
        Cw = -((Gw * motor.Kt) / (motor.Kv * motor.R * J_w))
        Vs = 0.5 * ((Gs * motor.Kt) / (motor.R * J_a))
        Vw = 0.5 * ((Gw * motor.Kt) / (motor.R * J_w))

        A = np.array([
            [0.0, 1.0, 0.0],
            [0.0, Gs * Cs, 0.0],
            [0.0, 0.0, Gw * Cw]
        ])

        B = np.array([
            [0.0, 0.0],
            [Vs, Vs],
            [Vw, -Vw],
        ])
        C = np.identity(3)
        D = np.zeros((3, 2))

        print(A)
        print(B)
        print(C)
        print(D)
        return ct.ss(A, B, C, D)

    def design_controller_observer(self):
        q = [
            Constants.Q_AZIMUTH,
            Constants.Q_AZIMUTH_ANG_VELOCITY,
            Constants.Q_WHEEL_ANG_VELOCITY,
        ]
        r = [Constants.VOLTAGE, Constants.VOLTAGE]
        self.design_lqr(q, r)
        self.design_two_state_feedforward()

        q_model = [
            Constants.MODEL_AZIMUTH_ANGLE_NOISE,
            Constants.MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
            Constants.MODEL_WHEEL_ANG_VELOCITY_NOISE,
        ]
        r_model = [
            Constants.SENSOR_AZIMUTH_ANGLE_NOISE,
            Constants.SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
            Constants.SENSOR_WHEEL_ANG_VELOCITY_NOISE,
        ]
        self.design_kalman_filter(q_model, r_model)


def main():
    dt = 0.005
    diff_swerve = DiffSwerve(dt)
    # flywheel.export_cpp_coeffs("DiffSwerve", "subsystems/")
    # flywheel.export_java_coeffs("DiffSwerve")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 15.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[1.0], [0.0], [1.0]])
        else:
            r = np.array([[0.0], [0.0], [0.0]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = diff_swerve.generate_time_responses(t, refs)
    diff_swerve.plot_time_responses(t, x_rec, ref_rec, u_rec)
    plt.gcf().set_size_inches(100, 50)

    if "--noninteractive" in sys.argv:
        plt.savefig("swerve_response.svg")
    else:
        plt.show()


if __name__ == "__main__":
    main()
