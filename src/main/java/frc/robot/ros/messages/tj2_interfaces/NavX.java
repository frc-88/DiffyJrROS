// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class NavX extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private frc.robot.ros.messages.geometry_msgs.Quaternion orientation = new frc.robot.ros.messages.geometry_msgs.Quaternion();
    private frc.robot.ros.messages.geometry_msgs.Vector3 angular_velocity = new frc.robot.ros.messages.geometry_msgs.Vector3();
    private frc.robot.ros.messages.geometry_msgs.Vector3 linear_acceleration = new frc.robot.ros.messages.geometry_msgs.Vector3();

    @Expose(serialize = false, deserialize = false)
    public final String _type = "tj2_interfaces/NavX";

    public NavX() {

    }

    public NavX(frc.robot.ros.messages.std_msgs.Header header, frc.robot.ros.messages.geometry_msgs.Quaternion orientation, frc.robot.ros.messages.geometry_msgs.Vector3 angular_velocity, frc.robot.ros.messages.geometry_msgs.Vector3 linear_acceleration) {
        this.header = header;
        this.orientation = orientation;
        this.angular_velocity = angular_velocity;
        this.linear_acceleration = linear_acceleration;
    }

    public NavX(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        this.orientation = new frc.robot.ros.messages.geometry_msgs.Quaternion(jsonObj.get("orientation").getAsJsonObject());
        this.angular_velocity = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("angular_velocity").getAsJsonObject());
        this.linear_acceleration = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("linear_acceleration").getAsJsonObject());
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public frc.robot.ros.messages.geometry_msgs.Quaternion getOrientation() {
        return this.orientation;
    }
    public frc.robot.ros.messages.geometry_msgs.Vector3 getAngularVelocity() {
        return this.angular_velocity;
    }
    public frc.robot.ros.messages.geometry_msgs.Vector3 getLinearAcceleration() {
        return this.linear_acceleration;
    }

    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setOrientation(frc.robot.ros.messages.geometry_msgs.Quaternion orientation) {
        this.orientation = orientation;
    }
    public void setAngularVelocity(frc.robot.ros.messages.geometry_msgs.Vector3 angular_velocity) {
        this.angular_velocity = angular_velocity;
    }
    public void setLinearAcceleration(frc.robot.ros.messages.geometry_msgs.Vector3 linear_acceleration) {
        this.linear_acceleration = linear_acceleration;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
