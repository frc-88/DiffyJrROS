package frc.robot.util.coprocessor.networktables;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;
import frc.robot.util.diffswerve.NavX;


public class DiffyJrTable extends CoprocessorTable {
    private DiffSwerveChassis swerve;
    private NavX imu;

    private NetworkTable targetTable;
    private NetworkTableEntry targetEntryDist;
    private NetworkTableEntry targetEntryAngle;
    private NetworkTableEntry targetEntryProbability;
    private NetworkTableEntry targetEntryUpdate;
    private double targetDistance = 0.0;
    private double targetAngle = 0.0;
    private double targetProbability = 0.0;
    private MessageTimer targetTimer = new MessageTimer(1_000_000);

    private final double kGravity = 9.81;

    private NetworkTableEntry fieldRelativeEntry;

    private NetworkTable moduleRootTable;
    private NetworkTableEntry moduleNumEntry;
    private ArrayList<NetworkTable> moduleTables = new ArrayList<>();

    private NetworkTable imuTable;
    private NetworkTableEntry imuEntryTx;  // filtered roll angle
    private NetworkTableEntry imuEntryTy;  // filtered pitch angle
    private NetworkTableEntry imuEntryTz;  // filtered yaw angle
    private NetworkTableEntry imuEntryVz;  // yaw rate
    private NetworkTableEntry imuEntryAx;  // linear accel x
    private NetworkTableEntry imuEntryAy;  // linear accel y
    private NetworkTableEntry imuEntryUpdate;


    public DiffyJrTable(DiffSwerveChassis swerve, NavX imu, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;
        this.imu = imu;

        targetTable = rootTable.getSubTable("target");
        targetEntryDist = targetTable.getEntry("distance");
        targetEntryAngle = targetTable.getEntry("heading");
        targetEntryProbability = targetTable.getEntry("probability");
        targetEntryUpdate = targetTable.getEntry("update");
        targetEntryUpdate.addListener(this::targetCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
        fieldRelativeEntry = rootTable.getEntry("field_relative");
        fieldRelativeEntry.addListener(this::fieldRelativeCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        moduleRootTable = rootTable.getSubTable("modules");
        moduleNumEntry = moduleRootTable.getEntry("num");
        for (int index = 0; index < this.swerve.getNumModules(); index++) {
            moduleTables.add(moduleRootTable.getSubTable(String.valueOf(index)));
        }
        moduleNumEntry.setDouble((double)this.swerve.getNumModules());

        imuTable = rootTable.getSubTable("imu");
        imuEntryTx = imuTable.getEntry("tx");
        imuEntryTy = imuTable.getEntry("ty");
        imuEntryTz = imuTable.getEntry("tz");
        imuEntryVz = imuTable.getEntry("vz");
        imuEntryAx = imuTable.getEntry("ax");
        imuEntryAy = imuTable.getEntry("ay");
        imuEntryUpdate = imuTable.getEntry("update");
    }

    @Override
    public void update() {
        super.update();

        updateImu();
        updateModules();
    }

    public void updateSlow() {
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            setJointPosition(index, state.angle.getRadians());
        }
    }

    private void updateModules() {
        for (int index = 0; index < this.swerve.getNumModules(); index++) {
            DiffSwerveModule module = this.swerve.getModule(index);
            NetworkTable moduleTable = moduleTables.get(index);
            moduleTable.getEntry("wheel_velocity").setDouble(module.getWheelVelocity());
            moduleTable.getEntry("azimuth_velocity").setDouble(module.getAzimuthVelocity());
            moduleTable.getEntry("azimuth").setDouble(module.getModuleAngle());
            moduleTable.getEntry("wheel_velocity_ref").setDouble(module.getReferenceWheelVelocity());
            moduleTable.getEntry("azimuth_velocity_ref").setDouble(module.getReferenceModuleAngularVelocity());
            moduleTable.getEntry("azimuth_ref").setDouble(module.getReferenceModuleAngle());
            moduleTable.getEntry("hi_voltage").setDouble(module.getHiMeasuredVoltage());
            moduleTable.getEntry("hi_voltage_ref").setDouble(module.getHiNextVoltage());
            moduleTable.getEntry("hi_velocity").setDouble(module.getHiRadiansPerSecond());
            moduleTable.getEntry("lo_voltage").setDouble(module.getLoMeasuredVoltage());
            moduleTable.getEntry("lo_voltage_ref").setDouble(module.getLoNextVoltage());
            moduleTable.getEntry("lo_velocity").setDouble(module.getLoRadiansPerSecond());
        }
    }

    public void updateImu()
    {
        imuEntryTx.setDouble(Units.degreesToRadians(imu.getRoll()));
        imuEntryTy.setDouble(Units.degreesToRadians(imu.getPitch()));
        imuEntryTz.setDouble(Units.degreesToRadians(imu.getYaw()));
        imuEntryVz.setDouble(Units.degreesToRadians(imu.getYawRate()));
        imuEntryAx.setDouble(imu.getAccelX() * kGravity);
        imuEntryAy.setDouble(imu.getAccelY() * kGravity);
        imuEntryUpdate.setDouble(getTime());
    }

    private void targetCallback(EntryNotification notification) {
        targetDistance = targetEntryDist.getDouble(0.0);
        targetAngle = targetEntryAngle.getDouble(0.0);
        targetProbability = targetEntryProbability.getDouble(0.0);
        targetTimer.reset();
    }

    private void fieldRelativeCallback(EntryNotification notification) {
        boolean value = notification.getEntry().getBoolean(false);
        if (value) {
            this.swerve.resetFieldOffset();
        }
        System.out.println("Setting field relative commands to " + value);
        this.swerve.setFieldRelativeCommands(value);
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
