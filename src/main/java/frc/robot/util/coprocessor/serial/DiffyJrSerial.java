package frc.robot.util.coprocessor.serial;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.coprocessor.tunnel.DataStreamInterface;
import frc.robot.util.coprocessor.tunnel.PacketResult;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;
import frc.robot.util.diffswerve.NavX;

public class DiffyJrSerial extends CoprocessorSerial {
    private DiffSwerveChassis swerve;
    private NavX imu;

    private double targetDistance = 0.0;
    private double targetAngle = 0.0;
    private double targetProbability = 0.0;
    private MessageTimer targetTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);

    public DiffyJrSerial(DiffSwerveChassis swerve, NavX imu) {
        super((ChassisInterface)swerve);
        this.swerve = swerve;
        this.imu = imu;
    }

    public void packetCallback(DataStreamInterface data_stream, PacketResult result)
    {
        super.packetCallback(data_stream, result);
        
        String category = result.getCategory();
        if (category.equals("target")) {
            
            Pair<Double, Boolean> targetDistancePair = result.getDouble(); if (!targetDistancePair.getSecond()) { System.out.println("Failed to get target distance"); return; }
            Pair<Double, Boolean> targetAnglePair = result.getDouble(); if (!targetAnglePair.getSecond()) { System.out.println("Failed to get target angle"); return; }
            Pair<Double, Boolean> targetProbabilityPair = result.getDouble(); if (!targetProbabilityPair.getSecond()) { System.out.println("Failed to get target probability"); return; }

            targetDistance = targetDistancePair.getFirst();
            targetAngle = targetAnglePair.getFirst();
            targetProbability = targetProbabilityPair.getFirst();
            targetTimer.reset();
        }
        else if (category.equals("relative")) {
            Pair<Integer, Boolean> value = result.getInt(); if (!value.getSecond()) { System.out.println("Failed to get joint index"); return; }
            boolean is_relative = value.getFirst() > 0 ? true : false;
            if (is_relative) {
                this.swerve.softResetImu();
            }
            System.out.println("Setting field relative commands to " + is_relative);
            this.swerve.setFieldRelativeCommands(is_relative);
        }
    }
    // public void update() {
    //     super.update();
    //     writeImu(imu.getBase());
    // }


    public void updateSlow() {
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            setJointPosition(index, state.angle.getRadians());

            data_stream.writePacket(
                "module", index,
                module.getWheelVelocity(),
                module.getAzimuthVelocity(),
                module.getModuleAngle(),
                module.getHiNextVoltage(),
                module.getLoNextVoltage(),
                module.getHiMeasuredVoltage(),
                module.getLoMeasuredVoltage(),
                module.getHiMeasuredCurrent(),
                module.getLoMeasuredCurrent()
            );
        }
    }

    public double getTargetDistance() {
        return targetDistance;
    }
    public double getTargetAngle() {
        return targetAngle;
    }
    public double getTargetProbability() {
        return targetProbability;
    }
    public boolean isTargetValid() {
        return targetTimer.isActive();
    }
}
