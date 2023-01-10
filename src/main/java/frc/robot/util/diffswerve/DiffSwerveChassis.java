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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.coprocessor.BoundingBox;
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
    private ChassisSpeeds chassisSpeedsSetpoint = new ChassisSpeeds();

    private final SlewRateLimiter batteryLimiter;

    private boolean fieldRelativeCommands = false;
    private Rotation2d fieldRelativeOffset = new Rotation2d();

    private SwerveModulePosition[] modulePositions;

    public final NavX imu;

    public final BoundingBox collisionBoundingBox;

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

        modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(imu.getYaw()), modulePositions);

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

        batteryLimiter = new SlewRateLimiter(Constants.DriveTrain.MAX_BATTERY_SLEW_RATE);
        batteryLimiter.reset(RobotController.getBatteryVoltage());

        collisionBoundingBox = new BoundingBox(
            Constants.BOUNDARY_WIDTH / 2.0, 
            Constants.BOUNDARY_LENGTH / 2.0,
            -Constants.BOUNDARY_WIDTH / 2.0,
            -Constants.BOUNDARY_LENGTH / 2.0,
            Constants.MIN_COLLISION_INFLATE,
            Constants.MAX_COLLISION_INFLATE,
            Constants.DriveTrain.MAX_CHASSIS_SPEED
        );

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

    public boolean commandsAreFieldRelative() {
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
        updateModulePositions();
        odometry.update(getHeading(), modulePositions);
        updateIdealState();
    }

    private void updateModulePositions() {
        modulePositions[0] = frontLeft.getPosition();
        modulePositions[1] = backLeft.getPosition();
        modulePositions[2] = backRight.getPosition();
        modulePositions[3] = frontRight.getPosition();
    }

    private void updateIdealState()
    {
        setIdealState(getModuleStatesWithConstraints(chassisSpeedsSetpoint));
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        chassisSpeedsSetpoint = speeds;
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
        frontLeft.resetPosition(new SwerveModulePosition());
        backLeft.resetPosition(new SwerveModulePosition());
        backRight.resetPosition(new SwerveModulePosition());
        frontRight.resetPosition(new SwerveModulePosition());
        updateModulePositions();
        odometry.resetPosition(
            getHeading(),
            modulePositions,
            pose
        );
    }

    public void resetFieldOffset() {
        fieldRelativeOffset = getHeading();
    }

    public NavX getImu() {
        return imu;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                backLeft.getState(),
                backRight.getState(),
                frontRight.getState());
    }

    public double getChassisSpeed() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.sqrt(
            speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + 
            speeds.vyMetersPerSecond * speeds.vyMetersPerSecond
        );
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

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(imu.getYaw());
        // return odometry.getPoseMeters().getRotation();
    }

    public Rotation2d getHeadingWithOffset() {
        return getHeading().minus(fieldRelativeOffset);
    }

    public Rotation2d getHeadingRate() {
        return Rotation2d.fromDegrees(imu.getYawRate());
        // return Rotation2d.fromDegrees(getChassisVelocity().omegaRadiansPerSecond);
    }

    public Rotation2d getAnglePidMeasurement() {
        return getHeadingWithOffset();
        // return getHeadingRate();
    }

    public void setIdealState(SwerveModuleState[] swerveModuleStates) {
        for (int index = 0; index < swerveModuleStates.length; index++) {
            modules[index].setIdealState(swerveModuleStates[index]);
        }
    }

    private void resetAngleSetpoint() {
        if (angleControllerEnabled) {
            this.angleSetpoint = getAnglePidMeasurement().getRadians();
            angleController.reset(this.angleSetpoint);
        }
    }

    public void drive(VelocityCommand command) {
        drive(command.vx, command.vy, command.vt);
    }

    public void drive(ChassisSpeeds command) {
        drive(command.vxMetersPerSecond, command.vyMetersPerSecond, command.omegaRadiansPerSecond);
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx              velocity in x direction
     * @param vy              velocity in y direction
     * @param angularVelocity angular velocity (rotating speed)
     */
    public void drive(double vx, double vy, double angularVelocity) {
        if (isWithinDeadband(vx, vy, angularVelocity)) {
            // if setpoints are almost zero, set chassis to hold position
            setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        } else if (!angleControllerEnabled || Math.abs(angularVelocity) > 0) {
            // if translation and rotation are significant, push setpoints as-is
            setChassisSpeeds(new ChassisSpeeds(vx, vy, angularVelocity));
            resetAngleSetpoint();
        } else {
            // if only translation is significant, set angular velocity according to
            // previous angle setpoint
            double controllerAngVel = angleController.calculate(getAnglePidMeasurement().getRadians(), angleSetpoint);
            setChassisSpeeds(new ChassisSpeeds(vx, vy, controllerAngVel));
        }
    }

    private boolean isWithinDeadband(double vx, double vy, double angularVelocity)
    {
        return Math.abs(vx) < Constants.DriveTrain.LINEAR_DEADBAND
                && Math.abs(vy) < Constants.DriveTrain.LINEAR_DEADBAND
                && Math.abs(angularVelocity) < Constants.DriveTrain.ANG_DEADBAND;
    }

    private boolean isWithinDeadband(ChassisSpeeds speeds) {
        return isWithinDeadband(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    private SwerveModuleState[] getModuleStatesWithConstraints(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds limitedChassisSpeeds = getAccelLimitedChassisSpeeds(chassisSpeeds);
        SwerveModuleState[] swerveModuleStates;

        if (commandsAreFieldRelative()) {
            // Apply field relative adjustment after slew limiter to avoid lagging
            limitedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                limitedChassisSpeeds.vxMetersPerSecond, 
                limitedChassisSpeeds.vyMetersPerSecond, 
                limitedChassisSpeeds.omegaRadiansPerSecond, 
                getHeadingWithOffset());
        }

        if (isWithinDeadband(limitedChassisSpeeds)) {
            // if within the command deadband, hold the module directions
            swerveModuleStates = new SwerveModuleState[modules.length];
            for (int index = 0; index < modules.length; index++) {
                swerveModuleStates[index] = new SwerveModuleState(0.0, new Rotation2d(modules[index].getModuleAngle()));
            }
            resetAngleSetpoint();
        }
        else {
            swerveModuleStates = kinematics.toSwerveModuleStates(limitedChassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getBatteryLimitedMaxSpeed());
        }

        return swerveModuleStates;
    }

    private double getBatteryLimitedMaxSpeed() {
        if (Constants.DriveTrain.ENABLE_BATTERY_CONSTRAINT) {
            double batteryVoltage = RobotController.getBatteryVoltage();
            double limitedBatteryVoltage = batteryLimiter.calculate(batteryVoltage);
            if (batteryVoltage < limitedBatteryVoltage) {
                batteryLimiter.reset(batteryVoltage);
                limitedBatteryVoltage = batteryVoltage;
            }

            if (limitedBatteryVoltage < Constants.DriveTrain.BROWNOUT_ZONE) {
                limitedBatteryVoltage = Constants.DriveTrain.BROWNOUT_ZONE_MAX_VOLTAGE;
            }

            double adjustedMaxSpeed = Constants.DriveTrain.MAX_CHASSIS_SPEED *
                limitedBatteryVoltage /
                    Constants.DifferentialSwerveModule.CONTROL_EFFORT;
            if (adjustedMaxSpeed > Constants.DriveTrain.MAX_CHASSIS_SPEED) {
                adjustedMaxSpeed = Constants.DriveTrain.MAX_CHASSIS_SPEED;
            } else if (adjustedMaxSpeed < 0.0) {
                adjustedMaxSpeed = 0.0;
            }
            return adjustedMaxSpeed;
        }
        else {
            return Constants.DriveTrain.MAX_CHASSIS_SPEED;
        }
    }

    private ChassisSpeeds getAccelLimitedChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
            Constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
                        ? vxLimiter.calculate(chassisSpeeds.vxMetersPerSecond)
                        : chassisSpeeds.vxMetersPerSecond,
            Constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
                        ? vyLimiter.calculate(chassisSpeeds.vyMetersPerSecond)
                        : chassisSpeeds.vyMetersPerSecond,
            Constants.DriveTrain.ENABLE_ANG_ACCEL_CONSTRAINT
                        ? vtLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond)
                        : chassisSpeeds.omegaRadiansPerSecond
        );
    }

    public void followPose(Pose2d pose, Rotation2d heading, double vel) {
        ChassisSpeeds adjustedSpeeds = controller.calculate(odometry.getPoseMeters(), pose, vel, heading);
        setChassisSpeeds(adjustedSpeeds);
    }

    @Override
    public void stop() {
        setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public void resetPosition(Pose2d pose) {
        resetOdom(pose);
    }

    @Override
    public BoundingBox getBoundingBox() {
        return collisionBoundingBox;
    }
}
