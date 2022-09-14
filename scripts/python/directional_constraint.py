import numpy as np
import scipy.stats
from matplotlib import pyplot as plt

def load_file(path):
    modules = []
    with open(path) as file:
        contents = file.read()
    for row in contents.splitlines():
        row = row.split("\t")
        try:
            index = int(row.pop(0))
            row = list(map(float, row))
            while len(modules) < index + 1:
                modules.append([])
            modules[index].append(row)
        except ValueError:
            continue
        
    return modules

def bound_half_angle(angle: np.array):
    angle %= np.pi * 2.0
    angle[angle > np.pi] -= np.pi * 2.0
    angle[angle < -np.pi] += np.pi * 2.0

def bound_quarter_angle(angle: np.array):
    angle %= np.pi
    angle[angle > np.pi * 0.5] -= np.pi
    angle[angle < -np.pi * 0.5] += np.pi

def gauss(x, mean, stddev):
    x1 = (x - mean) / stddev
    logStandardDeviationPlusHalfLog2Pi = np.log(stddev) + 0.5 * np.log(2 * np.pi)
    logDensity = -0.5 * x1 * x1 - logStandardDeviationPlusHalfLog2Pi
    sample = np.exp(logDensity)
    return sample

# setpoint
# measurement
# chassisSpeeds.vxMetersPerSecond
# chassisSpeeds.vyMetersPerSecond
# chassisSpeeds.omegaRadiansPerSecond

# path = "data/module_data_slow.txt"
# path = "data/module_data_medium.txt"
# path = "data/module_data_fast.txt"
path = "data/module_data_twirl.txt"
data = load_file(path)
module_0 = np.array(data[0])

setpoints = module_0[:, 0]
measurements = module_0[:, 1]

error = setpoints - measurements
setpoints[np.abs(error) > np.pi * 0.5] += np.pi
# bound_quarter_angle(setpoints)
# error = setpoints - measurements
bound_quarter_angle(error)

stddev = 0.6
normalize_const = gauss(0.0, 0.0, stddev)
cost = gauss(error / (np.pi * 0.5), 0.0, stddev) / normalize_const
# normalize_const = scipy.stats.norm.pdf(0.0, scale=stddev)
# cost = scipy.stats.norm.pdf(error / (np.pi * 0.5), scale=stddev) / normalize_const

plt.plot(setpoints, label="setpoint")
plt.plot(measurements, label="measurement")
plt.plot(cost, label="cost")
plt.plot(error, label="error")
plt.legend()

# x = np.linspace(-4, 4, 100)
# plt.plot(x, scipy.stats.norm.pdf(x, scale=0.5))
plt.show()
