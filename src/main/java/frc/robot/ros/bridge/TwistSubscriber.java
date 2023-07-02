package frc.robot.ros.bridge;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.geometry_msgs.Twist;

public class TwistSubscriber implements Subscriber<ChassisSpeeds> {
    private final BridgeSubscriber<Twist> twistSub;

    public TwistSubscriber(ROSNetworkTablesBridge bridge) {
        twistSub = new BridgeSubscriber<>(bridge, "cmd_vel", Twist.class);
    }

    public Optional<ChassisSpeeds> receive() {
        Twist msg;
        if ((msg = twistSub.receive()) != null) {
            return Optional.of(new ChassisSpeeds(msg.getLinear().getX(), msg.getLinear().getY(),
                    msg.getAngular().getZ()));
        } else {
            return Optional.empty();
        }
    }
}
