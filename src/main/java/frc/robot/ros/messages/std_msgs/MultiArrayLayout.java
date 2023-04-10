// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import java.util.ArrayList;
import java.util.Arrays;
import com.google.gson.JsonObject;

public class MultiArrayLayout implements frc.robot.ros.messages.RosMessage {

    private ArrayList<frc.robot.ros.messages.std_msgs.MultiArrayDimension> dim = new ArrayList<>();
    private int data_offset = 0;

    Gson ginst = new Gson();

    public MultiArrayLayout() {

    }

    public MultiArrayLayout(frc.robot.ros.messages.std_msgs.MultiArrayDimension[] dim, int data_offset) {
        this.dim = new ArrayList<>(Arrays.asList(dim));
        this.data_offset = data_offset;
    }

    public MultiArrayLayout(JsonObject jsonObj) {
        for (JsonElement dim_element : jsonObj.getAsJsonArray("dim")) {
            this.dim.add(new frc.robot.ros.messages.std_msgs.MultiArrayDimension(dim_element.getAsJsonObject()));
        }
        this.data_offset = jsonObj.get("data_offset").getAsInt();
    }

    public ArrayList<frc.robot.ros.messages.std_msgs.MultiArrayDimension> getDim() {
        return this.dim;
    }
    public int getDataOffset() {
        return this.data_offset;
    }

    public void setDim(ArrayList<frc.robot.ros.messages.std_msgs.MultiArrayDimension> dim) {
        this.dim = dim;
    }
    public void setDataOffset(int data_offset) {
        this.data_offset = data_offset;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
