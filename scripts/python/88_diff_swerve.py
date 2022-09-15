#!/usr/bin/env python3

import os
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as fct
import control as ct

import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal


class Constants:
    VOLTAGE = 12.0
    J_w = 0.015
    J_a = 0.014
    DIFF_MATRIX = np.array([
        [1.0/24.0, -1.0/24.0],
        [5.0/72.0, 7.0/72.0],
    ])
    Q_AZIMUTH = 0.0009  # radians
    Q_AZIMUTH_ANG_VELOCITY = 1.1  # radians per sec
    Q_WHEEL_ANG_VELOCITY = 0.001  # radians per sec

    MODEL_AZIMUTH_ANGLE_NOISE = 0.1  # radians
    MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0  # radians per sec
    MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0  # radians per sec

    SENSOR_AZIMUTH_ANGLE_NOISE = 0.5  # radians
    SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1  # radians per sec
    SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1  # radians per sec

    INV_DIFF_MATRIX = np.linalg.pinv(DIFF_MATRIX)

    MAX_SPEED = 4.48

    # GEAR_RATIO_WHEEL = 6.46875
    # GEAR_RATIO_STEER = 9.2
    # INV_DIFF_MATRIX = np.array([
    #     [GEAR_RATIO_WHEEL, GEAR_RATIO_WHEEL],
    #     [GEAR_RATIO_STEER, -GEAR_RATIO_STEER]
    # ])

    DEBUG_PRINTS = True

def input_modulus(input_num, minimum=-math.pi, maximum=math.pi):
    modulus = maximum - minimum
    # return input_num % modulus - minimum
    num_max = np.int32((input_num - minimum) / modulus)
    input_num -= num_max * modulus

    num_min = np.int32((input_num - maximum) / modulus)
    input_num -= num_min * modulus

    return input_num


class DiffSwerve(fct.System):
    def __init__(self, dt):
        """Diff Swerve subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Azimuth", "rad"), ("Azimuth velocity", "rad/s"), ("Wheel velocity", "rad/s")]
        u_labels = [("Lo Voltage", "V"), ("Hi Voltage", "V")]
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
        K_t = motor.Kt
        K_v = motor.Kv
        R = motor.R / 2.0
        J_w = Constants.J_w
        J_a = Constants.J_a
        
        A_subset = -K_t / (K_v * R * J_w) * np.dot(Constants.INV_DIFF_MATRIX, Constants.INV_DIFF_MATRIX)
        B_subset = K_t / (R * J_w) * Constants.INV_DIFF_MATRIX

        A = np.array([
            [0.0, 1.0, 0.0],
            [0.0, A_subset[0][0], A_subset[0][1]],
            [0.0, A_subset[1][0], A_subset[1][1]]
        ])

        B = np.array([
            [0.0, 0.0],
            [B_subset[0][0], B_subset[0][1]],
            [B_subset[1][0], B_subset[1][1]],
        ])
        # C = np.identity(3)
        C = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        D = np.zeros((3, 2))

        if Constants.DEBUG_PRINTS:
            print("A:\n", A)
            print("B:\n", B)
            print("C:\n", C)
            print("D:\n", D)
            print("J_w:", J_w)
            print("K_t:", K_t)
            print("K_v:", K_v)
            print("R:", R)
            print("Inv diff matrix:\n", Constants.INV_DIFF_MATRIX)
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

    def update(self, next_r=None):
        """Advance the model by one timestep.

        Keyword arguments:
        next_r -- next controller reference (default: current reference)
        """
        # update_plant
        # self.x = self.sysd.A @ self.x + self.sysd.B @ self.u
        # self.y = self.sysd.C @ self.x + self.sysd.D @ self.u
        self.update_plant()
        
        # correct_observer
        # self.y[0][0] = input_modulus(self.y[0][0]) * np.random.rand(1) / (Constants.SENSOR_AZIMUTH_ANGLE_NOISE ** 2)
        # self.x_hat += self.kalman_gain @ (
        #     self.y - self.sysd.C @ self.x_hat - self.sysd.D @ self.u
        # )
        self.correct_observer()

        # update_controller
        error = self.r - self.x_hat
        angle_error = input_modulus(error[0][0])
        error[0][0] = angle_error
        u = self.K @ error
        uff = self.Kff @ (next_r - self.sysd.A @ self.r)
        self.r = next_r
        self.u = np.clip(u + uff, self.u_min, self.u_max)

        # predict_observer
        # self.x_hat = self.sysd.A @ self.x_hat + self.sysd.B @ self.u
        self.predict_observer()


def get_phase_spectrum(yf):
    return np.angle(yf)
    # return np.unwrap(np.angle(yf))


def next_experiment(q_azimuth_sweep, q_wheel_ang_sweep, dt):
    q_azimuth_x = np.linspace(q_azimuth_sweep[0], q_azimuth_sweep[1], q_azimuth_sweep[2])
    q_wheel_ang_x = np.linspace(q_wheel_ang_sweep[0], q_wheel_ang_sweep[1], q_wheel_ang_sweep[2])
    print(f"Running {len(q_azimuth_x) * len(q_wheel_ang_x)} experiments")
    for q_azimuth in q_azimuth_x:
        for q_wheel_ang in q_wheel_ang_x:
            Constants.Q_AZIMUTH = q_azimuth
            Constants.Q_WHEEL_ANG_VELOCITY = q_wheel_ang

            diff_swerve = DiffSwerve(dt)

            yield q_azimuth, q_wheel_ang, diff_swerve

def run_sweep_experiment():
    Constants.DEBUG_PRINTS = False
    dt = 0.005
    q_azimuth_sweep = [0.001, 0.0015, 3]
    q_wheel_ang_sweep = [0.00001, 0.001, 10]

    l0 = 0.1
    l1 = l0 + 15.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 1.0, dt)

    refs = []

    start_freq = 0.1
    stop_freq = 5.0
    angles = scipy.signal.chirp(t - l0, start_freq, t[-1], stop_freq, phi=90.0) * np.pi
    velocities = np.ones(t.shape) * Constants.MAX_SPEED
    # velocities = np.zeros(t.shape)

    figures_dir = "./figures"
    if not os.path.isdir(figures_dir):
        os.makedirs(figures_dir)
    figure_path = os.path.join(figures_dir, "sweep-%0.4f-%0.4f.png")

    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[input_modulus(angles[i])], [0.0], [velocities[i]]])
        else:
            r = np.array([[0.0], [0.0], [0.0]])
        refs.append(r)

    min_q_azimuth = None
    min_q_wheel_ang = None
    min_delta = None
    for q_azimuth, q_wheel_ang, diff_swerve in next_experiment(q_azimuth_sweep, q_wheel_ang_sweep, dt):
        x_rec, ref_rec, u_rec, y_rec = diff_swerve.generate_time_responses(t, refs)
        # x_rec[0] = input_modulus(x_rec[0])

        # freq = np.fft.fftfreq(t.size, d=dt)
        # azimuth_meas_f = np.fft.fft(x_rec[0])
        # azimuth_set_f = np.fft.fft(ref_rec[0])

        # wheel_ang_meas_f = np.fft.fft(x_rec[2])
        # wheel_ang_set_f = np.fft.fft(ref_rec[2])

        # phase_delta = get_phase_spectrum(azimuth_set_f[0:t.size//2]) - get_phase_spectrum(azimuth_meas_f[0:t.size//2])
        worst_delta = np.max(ref_rec[0] - x_rec[0])
        if min_delta is None or worst_delta < min_delta:
            min_q_azimuth = q_azimuth
            min_q_wheel_ang = q_wheel_ang
            min_delta = worst_delta
        print(f"q_azimuth: {q_azimuth}, q_wheel_ang: {q_wheel_ang}, Max delta: {worst_delta}")
        diff_swerve.plot_time_responses(t, x_rec, ref_rec, u_rec)
        plt.gcf().set_size_inches(20, 10)
        plt.savefig(figure_path % (q_azimuth, q_wheel_ang))
        plt.close()
    print(f"min_q_azimuth: {min_q_azimuth}, min_q_wheel_ang: {min_q_wheel_ang}, min_delta: {min_delta}")


def main():
    dt = 0.005
    Constants.Q_AZIMUTH = 0.0009
    Constants.Q_WHEEL_ANG_VELOCITY = 0.001
    diff_swerve = DiffSwerve(dt)
    # diff_swerve.export_cpp_coeffs("DiffSwerve", "subsystems/")
    # diff_swerve.export_java_coeffs("DiffSwerve")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 15.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # angles = np.linspace(0.0, 12.0, len(t) // 2)
    # angles = np.append(angles, np.linspace(angles[-1], 0.0, len(t) - len(angles)))

    # velocities = np.linspace(0.0, 1.0, len(t) // 2)
    # velocities = np.append(velocities, np.linspace(velocities[-1], 0.0, len(t) - len(velocities)))

    # angles = np.sin(10 * t)
    start_freq = 0.1
    stop_freq = 5.0
    angles = scipy.signal.chirp(t - l0, start_freq, t[-1], stop_freq, phi=90.0) * np.pi
    velocities = scipy.signal.chirp(t, start_freq, t[-1], stop_freq) * Constants.MAX_SPEED
    # velocities = np.ones(t.shape) * Constants.MAX_SPEED
    # velocities = np.zeros(t.shape)

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0], [0.0]])
        elif t[i] < l1:
            # r = np.array([[input_modulus(angles[i])], [0.0], [4.48]])
            # r = np.array([[0.0], [0.0], [velocities[i]]])
            r = np.array([[input_modulus(angles[i])], [0.0], [velocities[i]]])
            # r = np.array([[0.001], [0.0], [0.0]])
            # r = np.array([[input_modulus(10.0)], [0.0], [4.48]])
            # r = np.array([[0.0], [0.0], [0.05]])
        else:
            r = np.array([[0.0], [0.0], [0.0]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = diff_swerve.generate_time_responses(t, refs)
    x_rec[0] = input_modulus(x_rec[0])
    diff_swerve.plot_time_responses(t, x_rec, ref_rec, u_rec)
    plt.gcf().set_size_inches(100, 50)

    plt.figure(2)
    angle_meas_f = np.fft.fft(x_rec[0])
    angle_set_f = np.fft.fft(ref_rec[0])
    freq = np.fft.fftfreq(t.size, d=dt)
    plt.subplot(2, 2, 1)
    plt.title("Amplitude spectrum angle")
    plt.plot(freq[0:t.size//2], 2/t.size*np.abs(angle_meas_f[0:t.size//2]), label='measured')
    plt.plot(freq[0:t.size//2], 2/t.size*np.abs(angle_set_f[0:t.size//2]), label='reference')
    plt.legend()
    plt.subplot(2, 2, 2)
    plt.title("Phase spectrum angle")
    plt.plot(freq[0:t.size//2], get_phase_spectrum(angle_set_f[0:t.size//2]) - get_phase_spectrum(angle_meas_f[0:t.size//2]), label='delta')
    plt.legend()

    vel_meas_f = np.fft.fft(x_rec[2])
    vel_set_f = np.fft.fft(ref_rec[2])
    freq = np.fft.fftfreq(t.size, d=dt)
    plt.subplot(2, 2, 3)
    plt.title("Amplitude spectrum velocity")
    plt.plot(freq[0:t.size//2], 2/t.size*np.abs(vel_meas_f[0:t.size//2]), label='measured')
    plt.plot(freq[0:t.size//2], 2/t.size*np.abs(vel_set_f[0:t.size//2]), label='reference')
    plt.legend()
    plt.subplot(2, 2, 4)
    plt.title("Phase spectrum velocity")
    plt.plot(freq[0:t.size//2], get_phase_spectrum(vel_set_f[0:t.size//2]) - get_phase_spectrum(vel_meas_f[0:t.size//2]), label='delta')
    plt.legend()
    # plt.phase_spectrum(x_rec[0])
    # plt.phase_spectrum(ref_rec[0])

    if "--noninteractive" in sys.argv:
        plt.savefig("swerve_response.svg")
    else:
        plt.show()


if __name__ == "__main__":
    main()
    # run_sweep_experiment()
