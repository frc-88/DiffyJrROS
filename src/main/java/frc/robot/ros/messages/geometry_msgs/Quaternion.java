// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

public class Quaternion implements frc.robot.ros.messages.RosMessage {

    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    private double w = 0.0;

    Gson ginst = new Gson();

    public Quaternion() {

    }

    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Quaternion(JsonObject jsonObj) {
        this.x = jsonObj.get("x").getAsDouble();
        this.y = jsonObj.get("y").getAsDouble();
        this.z = jsonObj.get("z").getAsDouble();
        this.w = jsonObj.get("w").getAsDouble();
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
    public double getW() {
        return this.w;
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
    public void setW(double w) {
        this.w = w;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}