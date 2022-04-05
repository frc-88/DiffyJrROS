package frc.robot.util.coprocessortable;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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

    public DiffyJrTable(DiffSwerveChassis swerve, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;

        targetTable = rootTable.getSubTable("target");
        targetEntryDist = targetTable.getEntry("distance");
        targetEntryAngle = targetTable.getEntry("heading");
        targetEntryProbability = targetTable.getEntry("probability");
        targetEntryUpdate = targetTable.getEntry("update");
        targetEntryUpdate.addListener(this::targetCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    // @Override
    // public void update() {
    //     super.update();
    // }

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
