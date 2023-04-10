// Auto generated!! Do not modify.
package frc.robot.ros.messages.visualization_msgs;

import java.util.ArrayList;
import com.google.gson.JsonObject;
import com.google.gson.JsonElement;
import java.util.Arrays;

public class Marker implements frc.robot.ros.messages.RosMessage {
    public static int ADD = 0;
    public static int POINTS = 8;
    public static int TRIANGLE_LIST = 11;
    public static int ARROW = 0;
    public static int SPHERE = 2;
    public static int CYLINDER = 3;
    public static int CUBE = 1;
    public static int TEXT_VIEW_FACING = 9;
    public static int MESH_RESOURCE = 10;
    public static int LINE_LIST = 5;
    public static int CUBE_LIST = 6;
    public static int DELETE = 2;
    public static int MODIFY = 0;
    public static int LINE_STRIP = 4;
    public static int DELETEALL = 3;
    public static int SPHERE_LIST = 7;

    private frc.robot.ros.messages.std_msgs.Header header = new frc.robot.ros.messages.std_msgs.Header();
    private String ns = "";
    private int id = 0;
    private int type = 0;
    private int action = 0;
    private frc.robot.ros.messages.geometry_msgs.Pose pose = new frc.robot.ros.messages.geometry_msgs.Pose();
    private frc.robot.ros.messages.geometry_msgs.Vector3 scale = new frc.robot.ros.messages.geometry_msgs.Vector3();
    private frc.robot.ros.messages.std_msgs.ColorRGBA color = new frc.robot.ros.messages.std_msgs.ColorRGBA();
    private long lifetime = 0;
    private boolean frame_locked = false;
    private ArrayList<frc.robot.ros.messages.geometry_msgs.Point> points = new ArrayList<>();
    private ArrayList<frc.robot.ros.messages.std_msgs.ColorRGBA> colors = new ArrayList<>();
    private String text = "";
    private String mesh_resource = "";
    private boolean mesh_use_embedded_materials = false;


    public Marker() {

    }

    public Marker(frc.robot.ros.messages.std_msgs.Header header, String ns, int id, int type, int action, frc.robot.ros.messages.geometry_msgs.Pose pose, frc.robot.ros.messages.geometry_msgs.Vector3 scale, frc.robot.ros.messages.std_msgs.ColorRGBA color, long lifetime, boolean frame_locked, frc.robot.ros.messages.geometry_msgs.Point[] points, frc.robot.ros.messages.std_msgs.ColorRGBA[] colors, String text, String mesh_resource, boolean mesh_use_embedded_materials) {
        this.header = header;
        this.ns = ns;
        this.id = id;
        this.type = type;
        this.action = action;
        this.pose = pose;
        this.scale = scale;
        this.color = color;
        this.lifetime = lifetime;
        this.frame_locked = frame_locked;
        this.points = new ArrayList<>(Arrays.asList(points));
        this.colors = new ArrayList<>(Arrays.asList(colors));
        this.text = text;
        this.mesh_resource = mesh_resource;
        this.mesh_use_embedded_materials = mesh_use_embedded_materials;
    }

    public Marker(JsonObject jsonObj) {
        this.header = new frc.robot.ros.messages.std_msgs.Header(jsonObj.get("header").getAsJsonObject());
        this.ns = jsonObj.get("ns").getAsString();
        this.id = jsonObj.get("id").getAsInt();
        this.type = jsonObj.get("type").getAsInt();
        this.action = jsonObj.get("action").getAsInt();
        this.pose = new frc.robot.ros.messages.geometry_msgs.Pose(jsonObj.get("pose").getAsJsonObject());
        this.scale = new frc.robot.ros.messages.geometry_msgs.Vector3(jsonObj.get("scale").getAsJsonObject());
        this.color = new frc.robot.ros.messages.std_msgs.ColorRGBA(jsonObj.get("color").getAsJsonObject());
        this.lifetime = jsonObj.get("lifetime").getAsLong();
        this.frame_locked = jsonObj.get("frame_locked").getAsBoolean();
        for (JsonElement points_element : jsonObj.getAsJsonArray("points")) {
            this.points.add(new frc.robot.ros.messages.geometry_msgs.Point(points_element.getAsJsonObject()));
        }
        for (JsonElement colors_element : jsonObj.getAsJsonArray("colors")) {
            this.colors.add(new frc.robot.ros.messages.std_msgs.ColorRGBA(colors_element.getAsJsonObject()));
        }
        this.text = jsonObj.get("text").getAsString();
        this.mesh_resource = jsonObj.get("mesh_resource").getAsString();
        this.mesh_use_embedded_materials = jsonObj.get("mesh_use_embedded_materials").getAsBoolean();
    }

    public frc.robot.ros.messages.std_msgs.Header getHeader() {
        return this.header;
    }

    public String getNs() {
        return this.ns;
    }

    public int getId() {
        return this.id;
    }

    public int getType() {
        return this.type;
    }

    public int getAction() {
        return this.action;
    }

    public frc.robot.ros.messages.geometry_msgs.Pose getPose() {
        return this.pose;
    }

    public frc.robot.ros.messages.geometry_msgs.Vector3 getScale() {
        return this.scale;
    }

    public frc.robot.ros.messages.std_msgs.ColorRGBA getColor() {
        return this.color;
    }

    public long getLifetime() {
        return this.lifetime;
    }

    public boolean getFrameLocked() {
        return this.frame_locked;
    }

    public ArrayList<frc.robot.ros.messages.geometry_msgs.Point> getPoints() {
        return this.points;
    }

    public ArrayList<frc.robot.ros.messages.std_msgs.ColorRGBA> getColors() {
        return this.colors;
    }

    public String getText() {
        return this.text;
    }

    public String getMeshResource() {
        return this.mesh_resource;
    }

    public boolean getMeshUseEmbeddedMaterials() {
        return this.mesh_use_embedded_materials;
    }



    public void setHeader(frc.robot.ros.messages.std_msgs.Header header) {
        this.header = header;
    }

    public void setNs(String ns) {
        this.ns = ns;
    }

    public void setId(int id) {
        this.id = id;
    }

    public void setType(int type) {
        this.type = type;
    }

    public void setAction(int action) {
        this.action = action;
    }

    public void setPose(frc.robot.ros.messages.geometry_msgs.Pose pose) {
        this.pose = pose;
    }

    public void setScale(frc.robot.ros.messages.geometry_msgs.Vector3 scale) {
        this.scale = scale;
    }

    public void setColor(frc.robot.ros.messages.std_msgs.ColorRGBA color) {
        this.color = color;
    }

    public void setLifetime(long lifetime) {
        this.lifetime = lifetime;
    }

    public void setFrameLocked(boolean frame_locked) {
        this.frame_locked = frame_locked;
    }

    public void setPoints(ArrayList<frc.robot.ros.messages.geometry_msgs.Point> points) {
        this.points = points;
    }

    public void setColors(ArrayList<frc.robot.ros.messages.std_msgs.ColorRGBA> colors) {
        this.colors = colors;
    }

    public void setText(String text) {
        this.text = text;
    }

    public void setMeshResource(String mesh_resource) {
        this.mesh_resource = mesh_resource;
    }

    public void setMeshUseEmbeddedMaterials(boolean mesh_use_embedded_materials) {
        this.mesh_use_embedded_materials = mesh_use_embedded_materials;
    }



    public String toString() {

        return "";
    }

    
}
