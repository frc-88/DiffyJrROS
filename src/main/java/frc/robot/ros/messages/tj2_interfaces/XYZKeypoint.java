// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;

public class XYZKeypoint extends frc.robot.ros.messages.RosMessage {

    private float x = 0.0f;
    private float y = 0.0f;
    private float z = 0.0f;

    public final String _type = "tj2_interfaces/XYZKeypoint";

    public XYZKeypoint() {

    }

    public XYZKeypoint(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public XYZKeypoint(JsonObject jsonObj) {
        this.x = jsonObj.get("x").getAsFloat();
        this.y = jsonObj.get("y").getAsFloat();
        this.z = jsonObj.get("z").getAsFloat();
    }

    public float getX() {
        return this.x;
    }
    public float getY() {
        return this.y;
    }
    public float getZ() {
        return this.z;
    }

    public void setX(float x) {
        this.x = x;
    }
    public void setY(float y) {
        this.y = y;
    }
    public void setZ(float z) {
        this.z = z;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
