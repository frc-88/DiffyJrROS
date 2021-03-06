package frc.robot.util.coprocessor.networktables;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;


public class DiffyJrTable extends CoprocessorTable {
    private DiffSwerveChassis swerve;

    private NetworkTable targetTable;
    private NetworkTableEntry targetEntryDist;
    private NetworkTableEntry targetEntryAngle;
    private NetworkTableEntry targetEntryProbability;
    private NetworkTableEntry targetEntryUpdate;
    private double targetDistance = 0.0;
    private double targetAngle = 0.0;
    private double targetProbability = 0.0;
    private MessageTimer targetTimer = new MessageTimer(1_000_000);

    private NetworkTableEntry fieldRelativeEntry;

    private NetworkTable moduleRootTable;
    private NetworkTableEntry moduleNumEntry;
    private ArrayList<NetworkTable> moduleTables = new ArrayList<>();

    public DiffyJrTable(DiffSwerveChassis swerve, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;

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
    }

    @Override
    public void update() {
        super.update();

        for (int index = 0; index < this.swerve.getNumModules(); index++) {
            moduleTables.get(index).getEntry("wheel_velocity").setDouble(this.swerve.getModule(index).getWheelVelocity());
            moduleTables.get(index).getEntry("azimuth_velocity").setDouble(this.swerve.getModule(index).getAzimuthVelocity());
            moduleTables.get(index).getEntry("azimuth").setDouble(this.swerve.getModule(index).getModuleAngle());
        }
    }

    public void updateSlow() {
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            setJointPosition(index, state.angle.getRadians());
        }
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
            this.swerve.softResetImu();
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
