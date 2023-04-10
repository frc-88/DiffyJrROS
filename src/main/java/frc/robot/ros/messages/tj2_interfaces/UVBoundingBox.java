// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;
import com.google.gson.JsonElement;

public class UVBoundingBox implements frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.tj2_interfaces.UVKeypoint[] points = new frc.robot.ros.messages.tj2_interfaces.UVKeypoint[] {
        new frc.robot.ros.messages.tj2_interfaces.UVKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.UVKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.UVKeypoint(),
        new frc.robot.ros.messages.tj2_interfaces.UVKeypoint()
    };
    private int width = 0;
    private int height = 0;


    public UVBoundingBox() {

    }

    public UVBoundingBox(frc.robot.ros.messages.tj2_interfaces.UVKeypoint[] points, int width, int height) {
        for (int index = 0; index < 4; index++) {
            this.points[index] = points[index];
        }
        this.width = width;
        this.height = height;
    }

    public UVBoundingBox(JsonObject jsonObj) {
        int points_element_index = 0;
        for (JsonElement points_element : jsonObj.getAsJsonArray("points")) {
            this.points[points_element_index] = new frc.robot.ros.messages.tj2_interfaces.UVKeypoint(points_element.getAsJsonObject());
        }
        this.width = jsonObj.get("width").getAsInt();
        this.height = jsonObj.get("height").getAsInt();
    }

    public frc.robot.ros.messages.tj2_interfaces.UVKeypoint[] getPoints() {
        return this.points;
    }

    public int getWidth() {
        return this.width;
    }

    public int getHeight() {
        return this.height;
    }



    public void setPoints(frc.robot.ros.messages.tj2_interfaces.UVKeypoint[] points) {
        this.points = points;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public void setHeight(int height) {
        this.height = height;
    }



    public String toString() {

        return "";
    }

    
}
