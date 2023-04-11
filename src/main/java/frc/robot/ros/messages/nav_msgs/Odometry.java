// Auto generated!! Do not modify.
package frc.robot.ros.messages.nav_msgs;

import com.google.gson.JsonObject;

public class Odometry extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private String child_frame_id = "";
    private frc.robot.ros.messages.geometry_msgs.PoseWithCovariance pose = new frc.robot.ros.messages.geometry_msgs.PoseWithCovariance();
    private frc.robot.ros.messages.geometry_msgs.TwistWithCovariance twist = new frc.robot.ros.messages.geometry_msgs.TwistWithCovariance();

    public final String _type = "nav_msgs/Odometry";

    public Odometry() {

    }

    public Odometry(frc.robot.ros.messages.std_msgs.Header header, String child_frame_id, frc.robot.ros.messages.geometry_msgs.PoseWithCovariance pose, frc.robot.ros.messages.geometry_msgs.TwistWithCovariance twist) {
        this.header = header;
        this.child_frame_id = child_frame_id;
        this.pose = pose;
        this.twist = twist;
    }

    public Odometry(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        this.child_frame_id = jsonObj.get("child_frame_id").getAsString();
        this.pose = new frc.robot.ros.messages.geometry_msgs.PoseWithCovariance(jsonObj.get("pose").getAsJsonObject());
        this.twist = new frc.robot.ros.messages.geometry_msgs.TwistWithCovariance(jsonObj.get("twist").getAsJsonObject());
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public String getChildFrameId() {
        return this.child_frame_id;
    }
    public frc.robot.ros.messages.geometry_msgs.PoseWithCovariance getPose() {
        return this.pose;
    }
    public frc.robot.ros.messages.geometry_msgs.TwistWithCovariance getTwist() {
        return this.twist;
    }

    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setChildFrameId(String child_frame_id) {
        this.child_frame_id = child_frame_id;
    }
    public void setPose(frc.robot.ros.messages.geometry_msgs.PoseWithCovariance pose) {
        this.pose = pose;
    }
    public void setTwist(frc.robot.ros.messages.geometry_msgs.TwistWithCovariance twist) {
        this.twist = twist;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
