// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.annotations.Expose;

public class XYZBoundingBox extends frc.team88.ros.messages.RosMessage {

    private frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] points = new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] {
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint()
    };
    private frc.team88.ros.messages.geometry_msgs.Vector3 dimensions = new frc.team88.ros.messages.geometry_msgs.Vector3();

    @Expose(serialize = false, deserialize = false)
    public final java.lang.String _type = "tj2_interfaces/XYZBoundingBox";

    public XYZBoundingBox() {

    }

    public XYZBoundingBox(frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] points, frc.team88.ros.messages.geometry_msgs.Vector3 dimensions) {
        for (int index = 0; index < 8; index++) {
            this.points[index] = points[index];
        }
        this.dimensions = dimensions;
    }

    public XYZBoundingBox(JsonObject jsonObj) {
        int points_element_index = 0;
        for (JsonElement points_element : jsonObj.getAsJsonArray("points")) {
            this.points[points_element_index] = new frc.robot.ros.messages.tj2_interfaces.XYZKeypoint(points_element.getAsJsonObject());
        }
        this.dimensions = new frc.team88.ros.messages.geometry_msgs.Vector3(jsonObj.get("dimensions").getAsJsonObject());
    }

    public frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] getPoints() {
        return this.points;
    }
    public frc.team88.ros.messages.geometry_msgs.Vector3 getDimensions() {
        return this.dimensions;
    }

    public void setPoints(frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] points) {
        this.points = points;
    }
    public void setDimensions(frc.team88.ros.messages.geometry_msgs.Vector3 dimensions) {
        this.dimensions = dimensions;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public java.lang.String toString() {
        return ginst.toJson(this);
    }
}
