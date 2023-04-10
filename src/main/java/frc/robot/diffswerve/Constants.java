package frc.robot.diffswerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double EPSILON = 0.00001;

    public static class DriveTrain {
        public static Pose2d robotPose;
        public static final double WIDTH = 0.30861;  // meters
        public static final double LENGTH = 0.30861;  // meters
        public static final double FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(140.449);  // module 0 offset degrees
        public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(-105.205);  // module 1 offset degrees
        public static final double BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(-46.055);  // module 2 offset degrees
        public static final double FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(-163.652);  // module 3 offset degrees
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);

        public static final double LINEAR_DEADBAND = 0.01;  // Linear command velocity deadband (m/s)
        public static final double ANG_DEADBAND = 0.01;  // Angular command velocity deadband (rad/s)

        public static final double MAX_CHASSIS_SPEED = 4.48;  // Maximum chassis speed (m/s)
        public static final double MAX_CHASSIS_ANG_VEL = 20.5;  // Maximum chassis rotational velocity (rad/s)
        public static final double MAX_CHASSIS_LINEAR_ACCEL = 6.0; // Maximum chassis linear acceleration (m/s^2)
        public static final double MAX_CHASSIS_ANG_ACCEL = 30.0; // Maximum chassis angular acceleration (m/s^2)
        public static final double MIN_CHASSIS_SPEED = 0.05;  // Minimum chassis speed that isn't zero (m/s)
        public static final double MIN_CHASSIS_ANG_VEL = 0.1;  // Minimum chassis rotational velocity that isn't zero (rad/s)

        public static final double MAX_BATTERY_SLEW_RATE = 2.0;  // How much to dampen the max voltage limit (Volts/second)
        public static final double BROWNOUT_ZONE = 7.0;  // Voltage below which velocities should be severely throttled
        public static final double BROWNOUT_ZONE_MAX_VOLTAGE = 2.5;  // Voltage scale to set while in the brown out zone
        public static final boolean ENABLE_BATTERY_CONSTRAINT = false;

        public static final double ANGLE_kP = 3.0;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.05;

        // Constraints
        public static final double CONSTRAINT_LINEAR_VEL = 4.5;  // PID Controller max velocity (m/s)
        public static final double CONSTRAINT_LINEAR_ACCEL = 12.0; // Maximum artificial linear acceleration (m/s^2)
        public static final double CONSTRAINT_ANG_ACCEL = 30.0; // Maximum artificial angular acceleration (m/s^2)
        public static final boolean ENABLE_LINEAR_ACCEL_CONSTRAINT = true;
        public static final boolean ENABLE_ANG_ACCEL_CONSTRAINT = false;

        public static final double CURVATURE_DT = 1.0 / 50.0;
        public static final double DIRECTIONAL_CONSTRAINT_STDDEV = 0.5;
        public static final double DIRECTIONAL_CONSTRAINT_DEADZONE = 0.95;
    }

    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        public static final double kDt = 0.005;


        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380.0);  // Radians per second
        public static final int TIMEOUT = 500;  // CAN sensor timeout
        
        // Differential swerve matrix constants. Matrix shape:
        // | M11   M12 |
        // | M21   M22 |
        public static final double GEAR_M11 = 1.0/24.0;
        public static final double GEAR_M12 = -1.0/24.0;
        public static final double GEAR_M21 = 5.0/72.0;
        public static final double GEAR_M22 = 7.0/72.0;

        public static final double FALCON_MAX_SPEED_RPS = Units.rotationsPerMinuteToRadiansPerSecond(600.0);  // radians per second
        public static final double WHEEL_RADIUS = 0.04445; // Meters with wheel compression.
        public static final double FALCON_TICKS_TO_ROTATIONS = 1.0 / 2048.0;  // rotations per tick
        public static final double AZIMUTH_ROTATIONS_TO_RADIANS = 2.0 * Math.PI;  // radians per rotation of azimuth sensor
        public static final double VOLTAGE = 12.0;  // volts
        public static final double FEED_FORWARD = VOLTAGE / FALCON_FREE_SPEED;
        public static final double NEUTRAL_DEADBAND_PERCENT = 0.001;

        public static final boolean ENABLE_CURRENT_LIMIT = false;
        public static final double CURRENT_LIMIT = 60.0;  // amps
        public static final double CURRENT_THRESHOLD = 60.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.003;  // kg * m^2
        // public static final double INERTIA_AZIMUTH = 0.005;  // kg * m^2
        
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH = 0.095; // radians
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_WHEEL_ANG_VELOCITY = 1.0; // radians per sec

        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our senlrs.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = 0.1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
        // the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.05; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.01; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.01; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    }
}
