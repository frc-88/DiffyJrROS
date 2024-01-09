package frc.robot.diffswerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

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
            talon.setNeutralMode(NeutralModeValue.Coast);
        } else {
            talon.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    private void checkErrorCode(StatusCode code) {
        if (code != StatusCode.OK) {
            throw new RuntimeException("Talon FX motor encountered an error: " + code.toString());
        }
    }

    private void initFalconMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
        config.Feedback.FeedbackRemoteSensorID = 0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.DifferentialSwerveModule.ENABLE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = Constants.DifferentialSwerveModule.CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentThreshold = Constants.DifferentialSwerveModule.CURRENT_THRESHOLD;
        config.CurrentLimits.SupplyTimeThreshold = Constants.DifferentialSwerveModule.CURRENT_TRIGGER_TIME;
        motor.getPosition().setUpdateFrequency(20);
        checkErrorCode(motor.getConfigurator().apply(config));
    }

    public void setVoltage(double voltage) {
        double limVoltage = Helpers.limit(
                voltage,
                -Constants.DifferentialSwerveModule.VOLTAGE,
                Constants.DifferentialSwerveModule.VOLTAGE);
        talon.set(limVoltage / Constants.DifferentialSwerveModule.VOLTAGE);
    }

    public double getVoltage() {
        return talon.getMotorVoltage().getValueAsDouble();
    }

    public double getCurrent() {
        return talon.getSupplyCurrent().getValueAsDouble();
    }

    public double getVelocityRadiansPerSecond() {
        return talon.getVelocity().getValueAsDouble()
                * Constants.DifferentialSwerveModule.FALCON_TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_MAX_SPEED_RPS;
    }

}
