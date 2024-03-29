// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.annotations.Expose;
import java.util.ArrayList;
import java.util.Arrays;

public class GameObjectsStamped extends frc.team88.ros.messages.RosMessage {

    private frc.team88.ros.messages.std_msgs.RosHeader header = new frc.team88.ros.messages.std_msgs.RosHeader();
    private ArrayList<frc.robot.ros.messages.tj2_interfaces.GameObject> objects = new ArrayList<>();
    private int width = 0;
    private int height = 0;

    @Expose(serialize = false, deserialize = false)
    public final java.lang.String _type = "tj2_interfaces/GameObjectsStamped";

    public GameObjectsStamped() {

    }

    public GameObjectsStamped(frc.team88.ros.messages.std_msgs.RosHeader header, frc.robot.ros.messages.tj2_interfaces.GameObject[] objects, int width, int height) {
        this.header = header;
        this.objects = new ArrayList<>(Arrays.asList(objects));
        this.width = width;
        this.height = height;
    }

    public GameObjectsStamped(JsonObject jsonObj) {
        this.header = new frc.team88.ros.messages.std_msgs.RosHeader(jsonObj.get("header").getAsJsonObject());
        for (JsonElement objects_element : jsonObj.getAsJsonArray("objects")) {
            this.objects.add(new frc.robot.ros.messages.tj2_interfaces.GameObject(objects_element.getAsJsonObject()));
        }
        this.width = jsonObj.get("width").getAsInt();
        this.height = jsonObj.get("height").getAsInt();
    }

    public frc.team88.ros.messages.std_msgs.RosHeader getHeader() {
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

    public void setHeader(frc.team88.ros.messages.std_msgs.RosHeader header) {
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

    public java.lang.String toString() {
        return ginst.toJson(this);
    }
}
