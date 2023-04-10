// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import java.util.ArrayList;
import java.util.Arrays;
import com.google.gson.JsonObject;

public class Float32MultiArray implements frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.MultiArrayLayout layout = new frc.robot.ros.messages.std_msgs.MultiArrayLayout();
    private ArrayList<Float> data = new ArrayList<>();

    Gson ginst = new Gson();

    public Float32MultiArray() {

    }

    public Float32MultiArray(frc.robot.ros.messages.std_msgs.MultiArrayLayout layout, Float[] data) {
        this.layout = layout;
        this.data = new ArrayList<>(Arrays.asList(data));
    }

    public Float32MultiArray(JsonObject jsonObj) {
        this.layout = new frc.robot.ros.messages.std_msgs.MultiArrayLayout(jsonObj.get("layout").getAsJsonObject());
        for (JsonElement data_element : jsonObj.getAsJsonArray("data")) {
            this.data.add(new Float(data_element.getAsFloat()));
        }
    }

    public frc.robot.ros.messages.std_msgs.MultiArrayLayout getLayout() {
        return this.layout;
    }
    public ArrayList<Float> getData() {
        return this.data;
    }

    public void setLayout(frc.robot.ros.messages.std_msgs.MultiArrayLayout layout) {
        this.layout = layout;
    }
    public void setData(ArrayList<Float> data) {
        this.data = data;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}