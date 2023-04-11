// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.JsonObject;

public class Vector3 extends frc.robot.ros.messages.RosMessage {

    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;

    public final String _type = "geometry_msgs/Vector3";

    public Vector3() {

    }

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3(JsonObject jsonObj) {
        this.x = jsonObj.get("x").getAsDouble();
        this.y = jsonObj.get("y").getAsDouble();
        this.z = jsonObj.get("z").getAsDouble();
    }

    public double getX() {
        return this.x;
    }
    public double getY() {
        return this.y;
    }
    public double getZ() {
        return this.z;
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setZ(double z) {
        this.z = z;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
