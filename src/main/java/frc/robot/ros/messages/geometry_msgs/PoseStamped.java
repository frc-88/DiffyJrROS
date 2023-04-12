// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class PoseStamped extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private frc.robot.ros.messages.geometry_msgs.Pose pose = new frc.robot.ros.messages.geometry_msgs.Pose();

    @Expose(serialize = false, deserialize = false)
    public final String _type = "geometry_msgs/PoseStamped";

    public PoseStamped() {

    }

    public PoseStamped(frc.robot.ros.messages.std_msgs.Header header, frc.robot.ros.messages.geometry_msgs.Pose pose) {
        this.header = header;
        this.pose = pose;
    }

    public PoseStamped(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        this.pose = new frc.robot.ros.messages.geometry_msgs.Pose(jsonObj.get("pose").getAsJsonObject());
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public frc.robot.ros.messages.geometry_msgs.Pose getPose() {
        return this.pose;
    }

    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setPose(frc.robot.ros.messages.geometry_msgs.Pose pose) {
        this.pose = pose;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
