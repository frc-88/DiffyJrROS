package frc.robot.util.coprocessortable;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;


public class DiffyJrTable extends CoprocessorTable {
    private DiffSwerveChassis swerve;

    public DiffyJrTable(DiffSwerveChassis swerve, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;
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
}
