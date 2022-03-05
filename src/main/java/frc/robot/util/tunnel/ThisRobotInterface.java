package frc.robot.util.tunnel;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.diffswerve.DiffSwerveChassis;
import frc.robot.util.diffswerve.DiffSwerveModule;

public class ThisRobotInterface extends ROSInterface {
    private DiffSwerveChassis swerve;

    public ThisRobotInterface(DiffSwerveChassis swerve) {
        super((ChassisInterface)swerve);
        this.swerve = swerve;
    }

    // @Override
    // public void packetCallback(TunnelClient tunnel, PacketResult result) {
    //     super.packetCallback(tunnel, result);
    // }

    @Override
    public void update() {
        super.update();
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            tunnel.writePacket("joint",
                index,
                state.angle.getRadians()
            );
        }
    }

    // @Override
    // public void updateSlow() {
    //     super.updateSlow();
    // }
}
