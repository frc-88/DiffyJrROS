package frc.robot.diffswerve;

import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;

public class DiffSwerveModule {
    private DiffSwerveMotor hiMotor;
    private DiffSwerveMotor loMotor;

    private final CANifiedPWMEncoder azimuthSensor;

    private final LinearSystemLoop<N3, N2, N3> swerveControlLoop;

    private Matrix<N3, N1> reference;
    private Matrix<N2, N1> input;

    private Matrix<N2, N2> diffMatrix;
    private Matrix<N2, N2> inverseDiffMatrix;

    private boolean is_enabled = false;

    private Translation2d moduleLocation;
    private double wheelPosition = 0.0;
    private double prevWheelPositionUpdateTime = 0.0;

    public DiffSwerveModule(
            Translation2d moduleLocation,
            int loCanID, int hiCanID, int azimuthCanifierID, int azimuthPwmChannel,
            double azimuthOffsetRadians) {
        loMotor = new DiffSwerveMotor(loCanID);
        hiMotor = new DiffSwerveMotor(hiCanID);

        azimuthSensor = new CANifiedPWMEncoder(
                azimuthCanifierID, azimuthPwmChannel, azimuthOffsetRadians,
                Constants.DifferentialSwerveModule.AZIMUTH_ROTATIONS_TO_RADIANS, false);

        diffMatrix = MatBuilder.fill(Nat.N2(), Nat.N2(),
                Constants.DifferentialSwerveModule.GEAR_M11, Constants.DifferentialSwerveModule.GEAR_M12,
                Constants.DifferentialSwerveModule.GEAR_M21, Constants.DifferentialSwerveModule.GEAR_M22);
        inverseDiffMatrix = diffMatrix.inv();

        swerveControlLoop = initControlLoop();
        input = VecBuilder.fill(0, 0);
        reference = MatBuilder.fill(Nat.N3(), Nat.N1(), 0, 0, 0);

        this.moduleLocation = moduleLocation;
    }

    public void setCoast(boolean coast) {
        hiMotor.setCoast(coast);
        loMotor.setCoast(coast);
    }

    public void setEnabled(boolean enabled) {
        is_enabled = enabled;
    }

    private LinearSystemLoop<N3, N2, N3> initControlLoop() {

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel = createDifferentialSwerveModule(DCMotor.getFalcon500(2));

        // Creates a Kalman Filter as our Observer for our module. Works since system is
        // linear.
        KalmanFilter<N3, N2, N3> swerveObserver = new KalmanFilter<>(
                Nat.N3(),
                Nat.N3(),
                swerveModuleModel,
                MatBuilder.fill(Nat.N3(), Nat.N1(),
                        Constants.DifferentialSwerveModule.MODEL_AZIMUTH_ANGLE_NOISE,
                        Constants.DifferentialSwerveModule.MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                        Constants.DifferentialSwerveModule.MODEL_WHEEL_ANG_VELOCITY_NOISE),
                MatBuilder.fill(Nat.N3(), Nat.N1(),
                        Constants.DifferentialSwerveModule.SENSOR_AZIMUTH_ANGLE_NOISE,
                        Constants.DifferentialSwerveModule.SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                        Constants.DifferentialSwerveModule.SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                Constants.DifferentialSwerveModule.kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N3> swerveController = new LinearQuadraticRegulator<>(
                swerveModuleModel,
                // Q Vector/Matrix Maximum error tolerance
                VecBuilder.fill(
                        Constants.DifferentialSwerveModule.Q_AZIMUTH,
                        Constants.DifferentialSwerveModule.Q_AZIMUTH_ANG_VELOCITY,
                        Constants.DifferentialSwerveModule.Q_WHEEL_ANG_VELOCITY),
                // R Vector/Matrix Maximum control effort.
                VecBuilder.fill(
                        Constants.DifferentialSwerveModule.CONTROL_EFFORT,
                        Constants.DifferentialSwerveModule.CONTROL_EFFORT),
                Constants.DifferentialSwerveModule.kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max
        // Volts,
        // Update Rate.
        LinearSystemLoop<N3, N2, N3> controlLoop = new LinearSystemLoop<>(
                swerveModuleModel,
                swerveController,
                swerveObserver,
                Constants.DifferentialSwerveModule.CONTROL_EFFORT,
                Constants.DifferentialSwerveModule.kDt);

        // Initializes the vectors and matrices.
        controlLoop.reset(VecBuilder.fill(0, 0, 0));

        return controlLoop;
    }

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @return LinearSystem of state space model.
     */
    private LinearSystem<N3, N2, N3> createDifferentialSwerveModule(DCMotor motor) {
        double J_w = Constants.DifferentialSwerveModule.INERTIA_WHEEL;
        // double J_a = Constants.DifferentialSwerveModule.INERTIA_AZIMUTH;
        double K_t = motor.KtNMPerAmp;
        double K_v = motor.KvRadPerSecPerVolt;
        double R = motor.rOhms;
        Matrix<N2, N2> A_subset = inverseDiffMatrix.times(inverseDiffMatrix).times(-K_t / (K_v * R * J_w));
        Matrix<N2, N2> B_subset = inverseDiffMatrix.times(K_t / (R * J_w));

        var A = MatBuilder.fill(Nat.N3(), Nat.N3(),
                0.0, 1.0, 0.0,
                0.0, A_subset.get(0, 0), A_subset.get(0, 1),
                0.0, A_subset.get(1, 0), A_subset.get(1, 1));

        var B = MatBuilder.fill(Nat.N3(), Nat.N2(),
                0.0, 0.0,
                B_subset.get(0, 0), B_subset.get(0, 1),
                B_subset.get(1, 0), B_subset.get(1, 1));
        var C = MatBuilder.fill(Nat.N3(), Nat.N3(),
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0);
        var D = MatBuilder.fill(Nat.N3(), Nat.N2(),
                0.0, 0.0,
                0.0, 0.0,
                0.0, 0.0);

        return new LinearSystem<>(A, B, C, D);
    }

    public void update() {
        // sets the next reference / setpoint.
        swerveControlLoop.setNextR(reference);
        // updates the kalman filter with new data points.
        Pair<Double, Double> velocities = getAngularVelocities();
        swerveControlLoop.correct(
                VecBuilder.fill(
                        getModuleAngle(), velocities.getFirst(), velocities.getSecond()));
        // predict step of kalman filter.
        if (!predict()) {
            return;
        }
        updateWheelPosition(getWheelVelocity());
        if (is_enabled) {
            hiMotor.setVoltage(getHiNextVoltage());
            loMotor.setVoltage(getLoNextVoltage());
        }
    }

    // use custom predict() function for as absolute encoder azimuth angle and the
    // angular velocity
    // of the module need to be continuous.
    private boolean predict() {
        // creates our input of voltage to our motors of u = K(r-x)+Kf*r but need to
        // wrap angle to be
        // continuous see computeErrorAndWrapAngle().

        Matrix<N2, N1> inputVelocities = getDifferentialInputs(reference.get(1, 0), reference.get(2, 0));
        Matrix<N2, N1> feedforwardVoltages = inputVelocities.times(Constants.DifferentialSwerveModule.FEED_FORWARD);

        input = swerveControlLoop.clampInput(
                swerveControlLoop
                        .getController()
                        .getK()
                        .times(
                                computeErrorAndWrapAngle(
                                        swerveControlLoop.getNextR(),
                                        swerveControlLoop.getXHat(), // can be NAN if measured wheel or azimuth is NaN
                                        -Math.PI,
                                        Math.PI))
                        .plus(feedforwardVoltages));
        if (Double.isNaN(input.get(0, 0)) || Double.isNaN(input.get(1, 0))) {
            System.out.println("Kalman observer input is NaN! Resetting controller.");
            swerveControlLoop.reset(VecBuilder.fill(0.0, 0.0, 0.0));
            return false;
        }

        swerveControlLoop.getObserver().predict(input, Constants.DifferentialSwerveModule.kDt);
        return true;
    }

    /**
     * wraps angle so that absolute encoder can be continues. (i.e) No issues when
     * switching between
     * -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as
     *                  [Math.PI, 0, 100].
     * @param xHat      is the predicted states of our system. [Azimuth Angle,
     *                  Azimuth Angular Velocity,
     *                  Wheel Angular Velocity].
     * @param minAngle  is the minimum angle in our case -PI.
     * @param maxAngle  is the maximum angle in our case PI.
     */
    private Matrix<N3, N1> computeErrorAndWrapAngle(
            Matrix<N3, N1> reference, Matrix<N3, N1> xHat, double minAngle, double maxAngle) {
        double angleError = reference.get(0, 0) - getModuleAngle();
        double positionError = MathUtil.inputModulus(angleError, minAngle, maxAngle);
        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
    }

    public Translation2d getModuleLocation() {
        return moduleLocation;
    }

    public double getModuleAngle() {
        return Helpers.boundHalfAngle(azimuthSensor.getPosition());
    }

    // Get module velocities. Pair order: (azimuth angular velocity, wheel angular
    // velocity)
    private Pair<Double, Double> getAngularVelocities() {
        Matrix<N2, N1> outputs = getDifferentialOutputs(
                loMotor.getVelocityRadiansPerSecond(),
                hiMotor.getVelocityRadiansPerSecond());
        return new Pair<Double, Double>(outputs.get(0, 0), outputs.get(1, 0));
    }

    // Get module wheel velocity in meters per sec.
    public double getWheelVelocity() {
        return getAngularVelocities().getSecond()
                * Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    private void updateWheelPosition(double wheelVelocity) {
        double currentTime = getTime();
        double dt = currentTime - prevWheelPositionUpdateTime;
        prevWheelPositionUpdateTime = currentTime;
        wheelPosition += wheelVelocity * dt;
    }

    private double getTime() {
        return RobotController.getFPGATime() * 1e-6;
    }

    public double getWheelPosition() {
        return wheelPosition;
    }

    // Get module azimuth velocity in radians per sec.
    public double getAzimuthVelocity() {
        return getAngularVelocities().getFirst();
    }

    public double getPredictedWheelVelocity() {
        return getPredictedWheelAngularVelocity() * Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    // Converts differential inputs (lo and hi motor rotational velocity -> azimuth
    // angular velocity, wheel angular velocity)
    public Matrix<N2, N1> getDifferentialOutputs(double angularVelocityLoMotor, double angularVelocityHiMotor) {
        return diffMatrix.times(VecBuilder.fill(angularVelocityLoMotor, angularVelocityHiMotor));
    }

    // Converts differential inputs (azimuth angular velocity, wheel angular
    // velocity -> lo and hi motor rotational velocity)
    public Matrix<N2, N1> getDifferentialInputs(double angularVelocityAzimuth, double angularVelocityWheel) {
        return inverseDiffMatrix.times(VecBuilder.fill(angularVelocityAzimuth, angularVelocityWheel));
    }

    public double getPredictedAzimuthAngularVelocity() {
        return swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return swerveControlLoop.getXHat(0);
    }

    private void setReference(Matrix<N3, N1> reference) {
        this.reference = reference;
    }

    /**
     * gets the wanted voltage from our control law. u = K(r-x) our control law is
     * slightly
     * different as we need to be continuous. Check method predict() for
     * calculations.
     *
     * @return hi wanted voltage
     */
    public double getHiNextVoltage() {
        return input.get(1, 0);
    }

    public double getLoNextVoltage() {
        return input.get(0, 0);
    }

    public double getReferenceModuleAngle() {
        return swerveControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return swerveControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return swerveControlLoop.getNextR(2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getModuleAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getWheelPosition(), new Rotation2d(getModuleAngle()));
    }

    public void resetPosition(SwerveModulePosition newPosition) {
        wheelPosition = newPosition.distanceMeters;
    }

    /**
     * Sets the state of the module and sends the voltages wanted to the motors.
     *
     * @param state is the desired swerve module state.
     */
    public void setModuleState(SwerveModuleState state) {
        setReference(
                VecBuilder.fill(
                        state.angle.getRadians(),
                        0.0,
                        // swerveControlLoop.getXHat(1),
                        state.speedMetersPerSecond
                                / Constants.DifferentialSwerveModule.WHEEL_RADIUS));
    }

    /**
     * sets the modules to take the shorted path to the newest state.
     *
     * @param state azimuth angle in radians and velocity of wheel in meters per
     *              sec.
     */
    public void setIdealState(SwerveModuleState state) {
        setModuleState(SwerveModuleState.optimize(state, new Rotation2d(getModuleAngle())));
    }
}
