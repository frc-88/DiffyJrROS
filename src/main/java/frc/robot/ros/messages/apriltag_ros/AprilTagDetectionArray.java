// Auto generated!! Do not modify.
package frc.robot.ros.messages.apriltag_ros;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.annotations.Expose;
import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagDetectionArray extends frc.team88.ros.messages.RosMessage {

    private frc.team88.ros.messages.std_msgs.Header header = new frc.team88.ros.messages.std_msgs.Header();
    private ArrayList<frc.robot.ros.messages.apriltag_ros.AprilTagDetection> detections = new ArrayList<>();

    @Expose(serialize = false, deserialize = false)
    public final java.lang.String _type = "apriltag_ros/AprilTagDetectionArray";

    public AprilTagDetectionArray() {

    }

    public AprilTagDetectionArray(frc.team88.ros.messages.std_msgs.Header header, frc.robot.ros.messages.apriltag_ros.AprilTagDetection[] detections) {
        this.header = header;
        this.detections = new ArrayList<>(Arrays.asList(detections));
    }

    public AprilTagDetectionArray(JsonObject jsonObj) {
        this.header = new frc.team88.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        for (JsonElement detections_element : jsonObj.getAsJsonArray("detections")) {
            this.detections.add(new frc.robot.ros.messages.apriltag_ros.AprilTagDetection(detections_element.getAsJsonObject()));
        }
    }

    public frc.team88.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public ArrayList<frc.robot.ros.messages.apriltag_ros.AprilTagDetection> getDetections() {
        return this.detections;
    }

    public void setHeader(frc.team88.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setDetections(ArrayList<frc.robot.ros.messages.apriltag_ros.AprilTagDetection> detections) {
        this.detections = detections;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public java.lang.String toString() {
        return ginst.toJson(this);
    }
}
