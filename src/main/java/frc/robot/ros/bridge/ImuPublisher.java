package frc.robot.ros.bridge;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.ros.messages.tj2_interfaces.NavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.geometry_msgs.Vector3;

public class ImuPublisher implements Publisher {
    private final frc.robot.diffswerve.NavX m_imu;
    private final BridgePublisher<NavX> m_imuPub;

    public ImuPublisher(DriveSubsystem drive, ROSNetworkTablesBridge bridge) {
        m_imu = drive.getImu();
        m_imuPub = new BridgePublisher<>(bridge, "imu");
    }

    public void publish() {
        m_imuPub.send(new NavX(m_imuPub.getHeader(Frames.IMU_FRAME),
                ROSConversions.wpiToRosRotation(new Rotation3d(m_imu.getRoll(), m_imu.getPitch(), m_imu.getYaw())),
                new Vector3(0.0, 0.0, m_imu.getYawRate()),
                new Vector3(m_imu.getAccelX(), m_imu.getAccelY(), m_imu.getAccelZ())));
    }
}
