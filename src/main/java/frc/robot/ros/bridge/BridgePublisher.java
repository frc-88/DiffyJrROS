package frc.robot.ros.bridge;

import java.util.Base64;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.ros.messages.RosMessage;
import frc.robot.ros.messages.Time;
import frc.robot.ros.messages.std_msgs.Header;

public class BridgePublisher<T extends RosMessage> {
    private ROSNetworkTablesBridge bridge;
    private String topicName;
    private StringPublisher pub = null;
    private int seq = 0;

    public BridgePublisher(ROSNetworkTablesBridge bridge, String topicName) {
        this.bridge = bridge;
        this.topicName = topicName;
    }

    public Header getHeader(String frame_id) {
        return new Header(getSeq(), getNow(), frame_id);
    }

    public Header getHeader(Time time, String frame_id) {
        return new Header(getSeq(), time, frame_id);
    }

    public Time getNow() {
        long localTime = RobotController.getFPGATime();
        int sec = (int) (localTime * 1e-6);
        int nsec = (int) (localTime * 1e3);
        return new Time(sec, nsec);
    }

    public int getSeq() {
        return seq++;
    }

    public void send(T msg) {
        if (this.pub == null) {
            this.pub = this.bridge.advertise(this.topicName);
        }
        String input = msg.toString();
        String encodedString = Base64.getEncoder().encodeToString(input.getBytes());
        this.pub.set(encodedString);
    }
}
