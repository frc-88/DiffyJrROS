package frc.robot.diffswerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DiffSwerveMotor {
    private TalonFX talon;
    private boolean is_ready;
    
    public DiffSwerveMotor(int canID) {
        do {
            try {
                talon = new TalonFX(canID);
                initFalconMotor(talon);
                is_ready = true;
            } catch (RuntimeException e) {
                System.out.println("Failed to set Falcon CAN parameter. Trying again: " + e);
            }
        } while (!is_ready);
    }

    public void setCoast(boolean coast) {
        if (coast) {
            talon.setNeutralMode(NeutralMode.Coast);
        } else {
            talon.setNeutralMode(NeutralMode.Brake);
        }
    }

    private void checkErrorCode(ErrorCode code) {
        if (code != ErrorCode.OK) {
            throw new RuntimeException("Talon FX motor encountered an error: " + code.toString());
        }
    }

    private void initFalconMotor(TalonFX motor) {
        checkErrorCode(motor.configFactoryDefault());
        motor.setInverted(false);
        motor.setSensorPhase(false);
        motor.setNeutralMode(NeutralMode.Brake);

        checkErrorCode(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configForwardSoftLimitEnable(false));
        checkErrorCode(motor.configVoltageCompSaturation(Constants.DifferentialSwerveModule.VOLTAGE,
                Constants.DifferentialSwerveModule.TIMEOUT));
        motor.enableVoltageCompensation(true);
        checkErrorCode(motor.setStatusFramePeriod(StatusFrame.Status_1_General, 5,
                Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20,
                Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configForwardSoftLimitEnable(false));
        checkErrorCode(motor.configNeutralDeadband(Constants.DifferentialSwerveModule.NEUTRAL_DEADBAND_PERCENT,
                Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configOpenloopRamp(0, Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configClosedloopRamp(0, Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        Constants.DifferentialSwerveModule.ENABLE_CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_THRESHOLD,
                        Constants.DifferentialSwerveModule.CURRENT_TRIGGER_TIME)));
        checkErrorCode(motor.configVoltageMeasurementFilter(0, Constants.DifferentialSwerveModule.TIMEOUT));
        checkErrorCode(motor.configMotionProfileTrajectoryInterpolationEnable(false,
                Constants.DifferentialSwerveModule.TIMEOUT));
    }

    public void setVoltage(double voltage) {
        double limVoltage = Helpers.limit(
                voltage,
                -Constants.DifferentialSwerveModule.VOLTAGE,
                Constants.DifferentialSwerveModule.VOLTAGE);
        talon.set(
                TalonFXControlMode.PercentOutput,
                limVoltage / Constants.DifferentialSwerveModule.VOLTAGE);
    }

    public double getVoltage() {
        return talon.getMotorOutputVoltage();
    }

    public double getCurrent() {
        return talon.getSupplyCurrent();
    }


    public double getVelocityRadiansPerSecond() {
        return talon.getSelectedSensorVelocity()
                * Constants.DifferentialSwerveModule.FALCON_TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_MAX_SPEED_RPS;
    }

}
