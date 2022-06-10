package frc.robot.util.diffswerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.00001;

    public static class DriveTrain {
        public static Pose2d robotPose;
        public static final double WIDTH = 0.30861;
        public static final double LENGTH = 0.30861;
        public static final double FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(141.240);
        public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(-105.469);
        public static final double BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(-48.164);
        public static final double FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(-168.038);
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);

        public static final double DEADBAND = 0.01;

        public static final double MAX_CHASSIS_SPEED = 4.48;  // Maximum chassis speed (m/s)
        public static final double MAX_CHASSIS_ANG_VEL = 20.5;  // Maximum chassis rotational velocity (rad/s)
        public static final double MAX_CHASSIS_LINEAR_ACCEL = 20.0; // Maximum chassis linear acceleration (m/s^2)
        public static final double MIN_CHASSIS_SPEED = 0.05;  // Minimum chassis speed that isn't zero (m/s)
        public static final double MIN_CHASSIS_ANG_VEL = 0.1;  // Minimum chassis rotational velocity that isn't zero (rad/s)

        public static final double ANGLE_kP = 3.0;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.05;
        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;

        // Constraints
        public static final double PROFILE_CONSTRAINT_VEL = 20.5;
        public static final double PROFILE_CONSTRAINT_ACCEL = 20.0;

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
        public static final double NEUTRAL_DEADBAND_PERCENT = 0.0;

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 60.0;  // amps
        public static final double CURRENT_THRESHOLD = 60.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.003;  // kg * m^2
        // public static final double INERTIA_AZIMUTH = 0.005;  // kg * m^2
        
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH = 0.1; // radians
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
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    }
}
