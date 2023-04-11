// Auto generated!! Do not modify.
package frc.robot.ros.messages.tf2_msgs;

import java.util.Arrays;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import com.google.gson.JsonElement;

public class TFMessage extends frc.robot.ros.messages.RosMessage {

    private ArrayList<frc.robot.ros.messages.geometry_msgs.TransformStamped> transforms = new ArrayList<>();

    public final String _type = "tf2_msgs/TFMessage";

    public TFMessage() {

    }

    public TFMessage(frc.robot.ros.messages.geometry_msgs.TransformStamped[] transforms) {
        this.transforms = new ArrayList<>(Arrays.asList(transforms));
    }

    public TFMessage(JsonObject jsonObj) {
        for (JsonElement transforms_element : jsonObj.getAsJsonArray("transforms")) {
            this.transforms.add(new frc.robot.ros.messages.geometry_msgs.TransformStamped(transforms_element.getAsJsonObject()));
        }
    }

    public ArrayList<frc.robot.ros.messages.geometry_msgs.TransformStamped> getTransforms() {
        return this.transforms;
    }

    public void setTransforms(ArrayList<frc.robot.ros.messages.geometry_msgs.TransformStamped> transforms) {
        this.transforms = transforms;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
