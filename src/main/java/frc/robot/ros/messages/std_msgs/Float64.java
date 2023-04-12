// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class Float64 extends frc.robot.ros.messages.RosMessage {

    private double data = 0.0;

    @Expose(serialize = false, deserialize = false)
    public final String _type = "std_msgs/Float64";

    public Float64() {

    }

    public Float64(double data) {
        this.data = data;
    }

    public Float64(JsonObject jsonObj) {
        this.data = jsonObj.get("data").getAsDouble();
    }

    public double getData() {
        return this.data;
    }

    public void setData(double data) {
        this.data = data;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
