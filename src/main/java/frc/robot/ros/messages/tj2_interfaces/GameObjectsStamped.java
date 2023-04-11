// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;
import java.util.Arrays;
import java.util.ArrayList;
import com.google.gson.JsonElement;

public class GameObjectsStamped extends frc.robot.ros.messages.RosMessage {

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private ArrayList<frc.robot.ros.messages.tj2_interfaces.GameObject> objects = new ArrayList<>();
    private int width = 0;
    private int height = 0;

    public final String _type = "tj2_interfaces/GameObjectsStamped";

    public GameObjectsStamped() {

    }

    public GameObjectsStamped(frc.robot.ros.messages.std_msgs.Header header, frc.robot.ros.messages.tj2_interfaces.GameObject[] objects, int width, int height) {
        this.header = header;
        this.objects = new ArrayList<>(Arrays.asList(objects));
        this.width = width;
        this.height = height;
    }

    public GameObjectsStamped(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        for (JsonElement objects_element : jsonObj.getAsJsonArray("objects")) {
            this.objects.add(new frc.robot.ros.messages.tj2_interfaces.GameObject(objects_element.getAsJsonObject()));
        }
        this.width = jsonObj.get("width").getAsInt();
        this.height = jsonObj.get("height").getAsInt();
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }
    public ArrayList<frc.robot.ros.messages.tj2_interfaces.GameObject> getObjects() {
        return this.objects;
    }
    public int getWidth() {
        return this.width;
    }
    public int getHeight() {
        return this.height;
    }

    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }
    public void setObjects(ArrayList<frc.robot.ros.messages.tj2_interfaces.GameObject> objects) {
        this.objects = objects;
    }
    public void setWidth(int width) {
        this.width = width;
    }
    public void setHeight(int height) {
        this.height = height;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
