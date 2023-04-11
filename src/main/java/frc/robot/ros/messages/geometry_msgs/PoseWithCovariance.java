// Auto generated!! Do not modify.
package frc.robot.ros.messages.geometry_msgs;

import com.google.gson.JsonObject;
import com.google.gson.JsonElement;

public class PoseWithCovariance extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.geometry_msgs.Pose pose = new frc.robot.ros.messages.geometry_msgs.Pose();
    private Double[] covariance = new Double[] {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    };

    public final String _type = "geometry_msgs/PoseWithCovariance";

    public PoseWithCovariance() {

    }

    public PoseWithCovariance(frc.robot.ros.messages.geometry_msgs.Pose pose, Double[] covariance) {
        this.pose = pose;
        for (int index = 0; index < 36; index++) {
            this.covariance[index] = covariance[index];
        }
    }

    public PoseWithCovariance(JsonObject jsonObj) {
        this.pose = new frc.robot.ros.messages.geometry_msgs.Pose(jsonObj.get("pose").getAsJsonObject());
        int covariance_element_index = 0;
        for (JsonElement covariance_element : jsonObj.getAsJsonArray("covariance")) {
            this.covariance[covariance_element_index] = covariance_element.getAsDouble();
        }
    }

    public frc.robot.ros.messages.geometry_msgs.Pose getPose() {
        return this.pose;
    }
    public Double[] getCovariance() {
        return this.covariance;
    }

    public void setPose(frc.robot.ros.messages.geometry_msgs.Pose pose) {
        this.pose = pose;
    }
    public void setCovariance(Double[] covariance) {
        this.covariance = covariance;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
