package frc.robot.util.coprocessor.networktables;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;
import frc.robot.util.diffswerve.NavX;


public class DiffyJrTable extends CoprocessorTable {
    private DiffSwerveChassis swerve;
    private NavX imu;

    private final double kGravity = 9.81;

    public DiffyJrTable(DiffSwerveChassis swerve, NavX imu, String address, int port, double updateInterval) {
        super((ChassisInterface)swerve, address, port, updateInterval);
        this.swerve = swerve;
        this.imu = imu;
    }

    @Override
    public void update() {
        super.update();
        sendImu(
            Units.degreesToRadians(imu.getRoll()),
            Units.degreesToRadians(imu.getPitch()),
            Units.degreesToRadians(imu.getYaw()),
            Units.degreesToRadians(imu.getYawRate()),
            imu.getAccelX() * kGravity,
            imu.getAccelY() * kGravity
        );
    }

    public void updateSlow() {
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            setJointPosition(index, state.angle.getRadians());
        }
        setNoGoZones(new String[] {"<!team>"});
    }
}
