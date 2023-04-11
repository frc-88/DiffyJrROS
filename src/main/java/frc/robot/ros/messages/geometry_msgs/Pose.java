// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.JsonObject;

public class Pose extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.geometry_msgs.Point position = new frc.robot.ros.messages.geometry_msgs.Point();
    private frc.robot.ros.messages.geometry_msgs.Quaternion orientation = new frc.robot.ros.messages.geometry_msgs.Quaternion();

    public final String _type = "geometry_msgs/Pose";

    public Pose() {

    }

    public Pose(frc.robot.ros.messages.geometry_msgs.Point position, frc.robot.ros.messages.geometry_msgs.Quaternion orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Pose(JsonObject jsonObj) {
        this.position = new frc.robot.ros.messages.geometry_msgs.Point(jsonObj.get("position").getAsJsonObject());
        this.orientation = new frc.robot.ros.messages.geometry_msgs.Quaternion(jsonObj.get("orientation").getAsJsonObject());
    }

    public frc.robot.ros.messages.geometry_msgs.Point getPosition() {
        return this.position;
    }
    public frc.robot.ros.messages.geometry_msgs.Quaternion getOrientation() {
        return this.orientation;
    }

    public void setPosition(frc.robot.ros.messages.geometry_msgs.Point position) {
        this.position = position;
    }
    public void setOrientation(frc.robot.ros.messages.geometry_msgs.Quaternion orientation) {
        this.orientation = orientation;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
