package frc.robot.util.diffswerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.VelocityCommand;

public class DiffSwerveChassis implements ChassisInterface {
    private final DiffSwerveModule frontLeft;
    private final DiffSwerveModule backLeft;
    private final DiffSwerveModule backRight;
    private final DiffSwerveModule frontRight;
    private final DiffSwerveModule[] modules;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final HolonomicDriveController controller;
    private final ProfiledPIDController angleController;
    private double angleSetpoint = 0.0;
    private boolean angleControllerEnabled = true;

    private final SlewRateLimiter vxLimiter;
    private final SlewRateLimiter vyLimiter;
    private final SlewRateLimiter vtLimiter;

    private boolean fieldRelativeCommands = false;
    private Rotation2d fieldRelativeImuOffset = new Rotation2d();

    public final NavX imu;

    private long timer = 0;

    public DiffSwerveChassis() {
        System.out.println("Creating diff swerve model");
        this.imu = new NavX();

        frontLeft = new DiffSwerveModule(
                Constants.DriveTrain.FRONT_LEFT_POSITION,
                RobotMap.CAN.TALONFX.FL_LO_FALCON,
                RobotMap.CAN.TALONFX.FL_HI_FALCON,
                RobotMap.CAN.CANIFIER,
                RobotMap.DIO.ENCODER_FL,
                Constants.DriveTrain.FRONT_LEFT_ENCODER_OFFSET);
        backLeft = new DiffSwerveModule(
                Constants.DriveTrain.BACK_LEFT_POSITION,
                RobotMap.CAN.TALONFX.BL_HI_FALCON,
                RobotMap.CAN.TALONFX.BL_LO_FALCON,
                RobotMap.CAN.CANIFIER,
                RobotMap.DIO.ENCODER_BL,
                Constants.DriveTrain.BACK_LEFT_ENCODER_OFFSET);
        backRight = new DiffSwerveModule(
                Constants.DriveTrain.BACK_RIGHT_POSITION,
                RobotMap.CAN.TALONFX.BR_LO_FALCON,
                RobotMap.CAN.TALONFX.BR_HI_FALCON,
                RobotMap.CAN.CANIFIER,
                RobotMap.DIO.ENCODER_BR,
                Constants.DriveTrain.BACK_RIGHT_ENCODER_OFFSET);
        frontRight = new DiffSwerveModule(
                Constants.DriveTrain.FRONT_RIGHT_POSITION,
                RobotMap.CAN.TALONFX.FR_LO_FALCON,
                RobotMap.CAN.TALONFX.FR_HI_FALCON,
                RobotMap.CAN.CANIFIER,
                RobotMap.DIO.ENCODER_FR,
                Constants.DriveTrain.FRONT_RIGHT_ENCODER_OFFSET);

        modules = new DiffSwerveModule[] { frontLeft, backLeft, backRight, frontRight };

        kinematics = new SwerveDriveKinematics(
                frontLeft.getModuleLocation(),
                backLeft.getModuleLocation(),
                backRight.getModuleLocation(),
                frontRight.getModuleLocation());
        odometry = new SwerveDriveOdometry(kinematics, getImuHeading());

        controller = new HolonomicDriveController(
                new PIDController(
                        Constants.DriveTrain.kP,
                        Constants.DriveTrain.kI,
                        Constants.DriveTrain.kD),
                new PIDController(
                        Constants.DriveTrain.kP,
                        Constants.DriveTrain.kI,
                        Constants.DriveTrain.kD),
                new ProfiledPIDController(
                        Constants.DriveTrain.kP,
                        Constants.DriveTrain.kI,
                        Constants.DriveTrain.kD,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveTrain.CONSTRAINT_LINEAR_VEL,
                                Constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL)));
        angleController = new ProfiledPIDController(
                Constants.DriveTrain.ANGLE_kP,
                Constants.DriveTrain.ANGLE_kI,
                Constants.DriveTrain.ANGLE_kD,
                new TrapezoidProfile.Constraints(
                        Constants.DriveTrain.CONSTRAINT_LINEAR_VEL,
                        Constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL));
        angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);

        vxLimiter = new SlewRateLimiter(Constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL);
        vyLimiter = new SlewRateLimiter(Constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL);
        vtLimiter = new SlewRateLimiter(Constants.DriveTrain.CONSTRAINT_ANG_ACCEL);

        System.out.println("Model created!");
    }

    public void setCoast(boolean coast) {
        for (DiffSwerveModule module : modules) {
            module.setCoast(coast);
        }
    }

    public void setFieldRelativeCommands(boolean isFieldRelative) {
        fieldRelativeCommands = isFieldRelative;
    }

    public boolean getFieldRelativeCommands() {
        return fieldRelativeCommands;
    }

    public void setEnabled(boolean is_enabled) {
        for (DiffSwerveModule module : modules) {
            module.setEnabled(is_enabled);
        }
    }

    public void setAngleControllerEnabled(boolean is_enabled) {
        this.angleControllerEnabled = is_enabled;
    }

    public void periodic() {
        // Called in main periodic callback in Robot
        odometry.update(
                getImuHeading(),
                frontLeft.getState(),
                backLeft.getState(),
                backRight.getState(),
                frontRight.getState());
    }

    public void controllerPeriodic() {
        // Called in separate periodic loop with a faster update rate
        for (DiffSwerveModule module : modules) {
            module.update();
        }
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdom(Pose2d pose) {
        odometry.resetPosition(pose, getImuHeading());
    }

    public void resetImu() {
        imu.reset();
        fieldRelativeImuOffset = new Rotation2d();
    }

    public void softResetImu() {
        fieldRelativeImuOffset = getImuHeading();
    }

    public NavX getImu() {
        return imu;
    }

    public ChassisSpeeds getChassisVelocity() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                backLeft.getState(),
                backRight.getState(),
                frontRight.getState());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        int index = 0;
        for (DiffSwerveModule module : modules) {
            states[index++] = module.getState();
        }
        return states;
    }

    public DiffSwerveModule getModule(int index) {
        return this.modules[index];
    }

    public DiffSwerveModule[] getModules() {
        return this.modules;
    }

    public int getNumModules() {
        return this.modules.length;
    }

    public Rotation2d getImuHeading() {
        return Rotation2d.fromDegrees(imu.getYaw());
    }

    public Rotation2d getImuHeadingWithOffset() {
        return getImuHeading().minus(fieldRelativeImuOffset);
    }

    public Rotation2d getImuHeadingRate() {
        return Rotation2d.fromDegrees(imu.getYawRate());
    }

    public Rotation2d getAnglePidMeasurement() {
        return getImuHeadingWithOffset();
        // return getImuHeadingRate();
    }

    // Set wheel velocities to zero and hold module directions
    public void holdDirection() {
        for (DiffSwerveModule module : modules) {
            module.setIdealState(new SwerveModuleState(0.0, new Rotation2d(module.getModuleAngle())));
        }
        resetAngleSetpoint();
        vxLimiter.reset(0.0);
        vyLimiter.reset(0.0);
        vtLimiter.reset(0.0);
    }

    public void setIdealState(SwerveModuleState[] swerveModuleStates) {
        for (int index = 0; index < swerveModuleStates.length; index++) {
            modules[index].setIdealState(swerveModuleStates[index]);
        }
    }

    private ChassisSpeeds getChassisSpeeds(double vx, double vy, double angularVelocity, boolean fieldRelative,
            Rotation2d relativeAngle) {
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, angularVelocity, relativeAngle);
        } else {
            chassisSpeeds = new ChassisSpeeds(vx, vy, angularVelocity);
        }
        return chassisSpeeds;
    }

    private void resetAngleSetpoint() {
        this.angleSetpoint = getAnglePidMeasurement().getRadians();
        angleController.reset(this.angleSetpoint);
    }

    public void drive(VelocityCommand command) {
        drive(command.vx, command.vy, command.vt);
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx              velocity in x direction
     * @param vy              velocity in y direction
     * @param angularVelocity angular velocity (rotating speed)
     */
    public void drive(double vx, double vy, double angularVelocity) {
        if (Math.abs(vx) < Constants.DriveTrain.DEADBAND
                && Math.abs(vy) < Constants.DriveTrain.DEADBAND
                && Math.abs(angularVelocity) < Constants.DriveTrain.DEADBAND) {
            // if setpoints are almost zero, set chassis to hold position
            holdDirection();
        } else if (!angleControllerEnabled || Math.abs(angularVelocity) > 0) {
            // if translation and rotation are significant, push setpoints as-is
            ChassisSpeeds chassisSpeeds = getChassisSpeeds(vx, vy, angularVelocity, getFieldRelativeCommands(),
                    getImuHeadingWithOffset());
            setIdealState(getModuleStatesWithConstraints(chassisSpeeds));
            resetAngleSetpoint();
        } else {
            // if only translation is significant, set angular velocity according to
            // previous angle setpoint
            double controllerAngVel = angleController.calculate(getAnglePidMeasurement().getRadians(), angleSetpoint);
            ChassisSpeeds chassisSpeeds = getChassisSpeeds(vx, vy, controllerAngVel, getFieldRelativeCommands(),
                    new Rotation2d(angleSetpoint));
            setIdealState(getModuleStatesWithConstraints(chassisSpeeds));
        }
    }

    private SwerveModuleState[] getModuleStatesWithConstraints(ChassisSpeeds chassisSpeeds) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double adjustedMaxSpeed = Constants.DriveTrain.MAX_CHASSIS_SPEED *
                batteryVoltage /
                Constants.DifferentialSwerveModule.CONTROL_EFFORT;
        if (adjustedMaxSpeed > Constants.DriveTrain.MAX_CHASSIS_SPEED) {
            adjustedMaxSpeed = Constants.DriveTrain.MAX_CHASSIS_SPEED;
        } else if (adjustedMaxSpeed < 0.0) {
            adjustedMaxSpeed = 0.0;
        }

        ChassisSpeeds limitedChassisSpeeds = getLimitedChassisSpeeds(chassisSpeeds);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(limitedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, adjustedMaxSpeed);

        return swerveModuleStates;
    }

    private ChassisSpeeds getLimitedChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
                Constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
                        ? vxLimiter.calculate(chassisSpeeds.vxMetersPerSecond)
                        : chassisSpeeds.vxMetersPerSecond,
                Constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
                        ? vyLimiter.calculate(chassisSpeeds.vyMetersPerSecond)
                        : chassisSpeeds.vyMetersPerSecond,
                Constants.DriveTrain.ENABLE_ANG_ACCEL_CONSTRAINT
                        ? vtLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond)
                        : chassisSpeeds.omegaRadiansPerSecond);
    }

    public void followPose(Pose2d pose, Rotation2d heading, double vel) {
        ChassisSpeeds adjustedSpeeds = controller.calculate(odometry.getPoseMeters(), pose, vel, heading);
        setIdealState(getModuleStatesWithConstraints(adjustedSpeeds));
    }

    @Override
    public void stop() {
        holdDirection();
    }

    @Override
    public void resetPosition(Pose2d pose) {
        resetOdom(pose);
    }
}
