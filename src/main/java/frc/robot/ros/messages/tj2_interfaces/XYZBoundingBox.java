// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;
import com.google.gson.JsonElement;

public class XYZBoundingBox implements frc.robot.ros.messages.RosMessage {

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
    private frc.robot.ros.messages.geometry_msgs.Vector3 dimensions = new frc.robot.ros.messages.geometry_msgs.Vector3();


    public XYZBoundingBox() {

    }

    public XYZBoundingBox(frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] points, frc.robot.ros.messages.geometry_msgs.Vector3 dimensions) {
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
        this.dimensions = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("dimensions").getAsJsonObject());
    }

    public frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] getPoints() {
        return this.points;
    }

    public frc.robot.ros.messages.geometry_msgs.Vector3 getDimensions() {
        return this.dimensions;
    }



    public void setPoints(frc.robot.ros.messages.tj2_interfaces.XYZKeypoint[] points) {
        this.points = points;
    }

    public void setDimensions(frc.robot.ros.messages.geometry_msgs.Vector3 dimensions) {
        this.dimensions = dimensions;
    }



    public String toString() {

        return "";
    }

    
}
