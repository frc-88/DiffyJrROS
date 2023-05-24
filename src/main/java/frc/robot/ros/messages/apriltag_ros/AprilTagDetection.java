// Auto generated!! Do not modify.
package frc.robot.ros.messages.apriltag_ros;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.annotations.Expose;
import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagDetection extends frc.team88.ros.messages.RosMessage {

    private ArrayList<java.lang.Integer> id = new ArrayList<>();
    private ArrayList<java.lang.Double> size = new ArrayList<>();
    private frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped pose = new frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped();

    @Expose(serialize = false, deserialize = false)
    public final java.lang.String _type = "apriltag_ros/AprilTagDetection";

    public AprilTagDetection() {

    }

    public AprilTagDetection(java.lang.Integer[] id, java.lang.Double[] size, frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped pose) {
        this.id = new ArrayList<>(Arrays.asList(id));
        this.size = new ArrayList<>(Arrays.asList(size));
        this.pose = pose;
    }

    public AprilTagDetection(JsonObject jsonObj) {
        for (JsonElement id_element : jsonObj.getAsJsonArray("id")) {
            this.id.add(id_element.getAsInt());
        }
        for (JsonElement size_element : jsonObj.getAsJsonArray("size")) {
            this.size.add(size_element.getAsDouble());
        }
        this.pose = new frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped(jsonObj.get("pose").getAsJsonObject());
    }

    public ArrayList<java.lang.Integer> getId() {
        return this.id;
    }
    public ArrayList<java.lang.Double> getSize() {
        return this.size;
    }
    public frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped getPose() {
        return this.pose;
    }

    public void setId(ArrayList<java.lang.Integer> id) {
        this.id = id;
    }
    public void setSize(ArrayList<java.lang.Double> size) {
        this.size = size;
    }
    public void setPose(frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped pose) {
        this.pose = pose;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public java.lang.String toString() {
        return ginst.toJson(this);
    }
}
