// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class Twist extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.geometry_msgs.Vector3 linear = new frc.robot.ros.messages.geometry_msgs.Vector3();
    private frc.robot.ros.messages.geometry_msgs.Vector3 angular = new frc.robot.ros.messages.geometry_msgs.Vector3();

    @Expose(serialize = false, deserialize = false)
    public final String _type = "geometry_msgs/Twist";

    public Twist() {

    }

    public Twist(frc.robot.ros.messages.geometry_msgs.Vector3 linear, frc.robot.ros.messages.geometry_msgs.Vector3 angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public Twist(JsonObject jsonObj) {
        this.linear = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("linear").getAsJsonObject());
        this.angular = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("angular").getAsJsonObject());
    }

    public frc.robot.ros.messages.geometry_msgs.Vector3 getLinear() {
        return this.linear;
    }
    public frc.robot.ros.messages.geometry_msgs.Vector3 getAngular() {
        return this.angular;
    }

    public void setLinear(frc.robot.ros.messages.geometry_msgs.Vector3 linear) {
        this.linear = linear;
    }
    public void setAngular(frc.robot.ros.messages.geometry_msgs.Vector3 angular) {
        this.angular = angular;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
