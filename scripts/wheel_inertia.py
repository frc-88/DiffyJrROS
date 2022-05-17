import math
from scipy import integrate

wheel_mass = 0.306  # kg
bearing_diameter = 0.0204  # meters
wheel_diameter = 0.0889  # meters

wheel_cross_section_area = 0.005880  # meters^2
wheel_length = 0.037592  # meters
wheel_volume = wheel_cross_section_area * wheel_length  # meters^3

wheel_density = wheel_mass / wheel_volume  # kg/m^3

def get_wheel_I(d, L, R1, R2):
    cylinder_integral, error = integrate.quad(lambda R: R**3, R1, R2)
    return d * L * 2.0 * math.pi * cylinder_integral


print(get_wheel_I(wheel_density, wheel_length, bearing_diameter / 2.0, wheel_diameter / 2.0))
