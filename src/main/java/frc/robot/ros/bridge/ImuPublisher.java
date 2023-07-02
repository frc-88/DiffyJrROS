package frc.robot.ros.bridge;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.ros.messages.tj2_interfaces.NavX;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.Vector3;

public class ImuPublisher implements Publisher {
    private final frc.robot.diffswerve.NavX imu;
    private final BridgePublisher<NavX> imuPub;

    public ImuPublisher(DriveSubsystem drive, ROSNetworkTablesBridge bridge) {
        imu = drive.getImu();
        imuPub = new BridgePublisher<>(bridge, "imu");
    }

    public void publish() {
        imuPub.send(new NavX(imuPub.getHeader(Frames.IMU_FRAME),
                ROSConversions.wpiToRosRotation(new Rotation3d(imu.getRoll(), imu.getPitch(), imu.getYaw())),
                new Vector3(0.0, 0.0, imu.getYawRate()),
                new Vector3(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ())));
    }
}
