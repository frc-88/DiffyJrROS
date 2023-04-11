// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.JsonObject;

public class TransformStamped extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private String child_frame_id = "";
    private frc.robot.ros.messages.geometry_msgs.Transform transform = new frc.robot.ros.messages.geometry_msgs.Transform();

    public final String _type = "geometry_msgs/TransformStamped";

    public TransformStamped() {

    }

    public TransformStamped(frc.robot.ros.messages.std_msgs.Header header, String child_frame_id, frc.robot.ros.messages.geometry_msgs.Transform transform) {
        this.header = header;
        this.child_frame_id = child_frame_id;
        this.transform = transform;
    }

    public TransformStamped(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        this.child_frame_id = jsonObj.get("child_frame_id").getAsString();
        this.transform = new frc.robot.ros.messages.geometry_msgs.Transform(jsonObj.get("transform").getAsJsonObject());
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public String getChildFrameId() {
        return this.child_frame_id;
    }
    public frc.robot.ros.messages.geometry_msgs.Transform getTransform() {
        return this.transform;
    }

    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setChildFrameId(String child_frame_id) {
        this.child_frame_id = child_frame_id;
    }
    public void setTransform(frc.robot.ros.messages.geometry_msgs.Transform transform) {
        this.transform = transform;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
