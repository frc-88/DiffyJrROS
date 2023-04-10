// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;

public class GameObject implements frc.robot.ros.messages.RosMessage {

    private String label = "";
    private int object_index = 0;
    private short class_index = 0;
    private float confidence = 0.0f;
    private frc.robot.ros.messages.geometry_msgs.Pose pose = new frc.robot.ros.messages.geometry_msgs.Pose();
    private frc.robot.ros.messages.tj2_interfaces.UVBoundingBox bounding_box_2d = new frc.robot.ros.messages.tj2_interfaces.UVBoundingBox();
    private frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox bounding_box_3d = new frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox();


    public GameObject() {

    }

    public GameObject(String label, int object_index, short class_index, float confidence, frc.robot.ros.messages.geometry_msgs.Pose pose, frc.robot.ros.messages.tj2_interfaces.UVBoundingBox bounding_box_2d, frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox bounding_box_3d) {
        this.label = label;
        this.object_index = object_index;
        this.class_index = class_index;
        this.confidence = confidence;
        this.pose = pose;
        this.bounding_box_2d = bounding_box_2d;
        this.bounding_box_3d = bounding_box_3d;
    }

    public GameObject(JsonObject jsonObj) {
        this.label = jsonObj.get("label").getAsString();
        this.object_index = jsonObj.get("object_index").getAsInt();
        this.class_index = jsonObj.get("class_index").getAsShort();
        this.confidence = jsonObj.get("confidence").getAsFloat();
        this.pose = new frc.robot.ros.messages.geometry_msgs.Pose(jsonObj.get("pose").getAsJsonObject());
        this.bounding_box_2d = new frc.robot.ros.messages.tj2_interfaces.UVBoundingBox(jsonObj.get("bounding_box_2d").getAsJsonObject());
        this.bounding_box_3d = new frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox(jsonObj.get("bounding_box_3d").getAsJsonObject());
    }

    public String getLabel() {
        return this.label;
    }

    public int getObjectIndex() {
        return this.object_index;
    }

    public short getClassIndex() {
        return this.class_index;
    }

    public float getConfidence() {
        return this.confidence;
    }

    public frc.robot.ros.messages.geometry_msgs.Pose getPose() {
        return this.pose;
    }

    public frc.robot.ros.messages.tj2_interfaces.UVBoundingBox getBoundingBox2D() {
        return this.bounding_box_2d;
    }

    public frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox getBoundingBox3D() {
        return this.bounding_box_3d;
    }



    public void setLabel(String label) {
        this.label = label;
    }

    public void setObjectIndex(int object_index) {
        this.object_index = object_index;
    }

    public void setClassIndex(short class_index) {
        this.class_index = class_index;
    }

    public void setConfidence(float confidence) {
        this.confidence = confidence;
    }

    public void setPose(frc.robot.ros.messages.geometry_msgs.Pose pose) {
        this.pose = pose;
    }

    public void setBoundingBox2D(frc.robot.ros.messages.tj2_interfaces.UVBoundingBox bounding_box_2d) {
        this.bounding_box_2d = bounding_box_2d;
    }

    public void setBoundingBox3D(frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox bounding_box_3d) {
        this.bounding_box_3d = bounding_box_3d;
    }



    public String toString() {

        return "";
    }

    
}
