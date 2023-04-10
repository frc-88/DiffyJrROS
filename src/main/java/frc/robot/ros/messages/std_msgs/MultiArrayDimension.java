// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

public class MultiArrayDimension implements frc.robot.ros.messages.RosMessage {

    private String label = "";
    private int size = 0;
    private int stride = 0;

    Gson ginst = new Gson();

    public MultiArrayDimension() {

    }

    public MultiArrayDimension(String label, int size, int stride) {
        this.label = label;
        this.size = size;
        this.stride = stride;
    }

    public MultiArrayDimension(JsonObject jsonObj) {
        this.label = jsonObj.get("label").getAsString();
        this.size = jsonObj.get("size").getAsInt();
        this.stride = jsonObj.get("stride").getAsInt();
    }

    public String getLabel() {
        return this.label;
    }
    public int getSize() {
        return this.size;
    }
    public int getStride() {
        return this.stride;
    }

    public void setLabel(String label) {
        this.label = label;
    }
    public void setSize(int size) {
        this.size = size;
    }
    public void setStride(int stride) {
        this.stride = stride;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
