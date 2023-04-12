// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class Transform extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.geometry_msgs.Vector3 translation = new frc.robot.ros.messages.geometry_msgs.Vector3();
    private frc.robot.ros.messages.geometry_msgs.Quaternion rotation = new frc.robot.ros.messages.geometry_msgs.Quaternion();

    @Expose(serialize = false, deserialize = false)
    public final String _type = "geometry_msgs/Transform";

    public Transform() {

    }

    public Transform(frc.robot.ros.messages.geometry_msgs.Vector3 translation, frc.robot.ros.messages.geometry_msgs.Quaternion rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Transform(JsonObject jsonObj) {
        this.translation = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("translation").getAsJsonObject());
        this.rotation = new frc.robot.ros.messages.geometry_msgs.Quaternion(jsonObj.get("rotation").getAsJsonObject());
    }

    public frc.robot.ros.messages.geometry_msgs.Vector3 getTranslation() {
        return this.translation;
    }
    public frc.robot.ros.messages.geometry_msgs.Quaternion getRotation() {
        return this.rotation;
    }

    public void setTranslation(frc.robot.ros.messages.geometry_msgs.Vector3 translation) {
        this.translation = translation;
    }
    public void setRotation(frc.robot.ros.messages.geometry_msgs.Quaternion rotation) {
        this.rotation = rotation;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
