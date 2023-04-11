// Auto generated!! Do not modify.
package frc.robot.ros.messages.visualization_msgs;

import com.google.gson.JsonObject;
import java.util.Arrays;
import java.util.ArrayList;
import com.google.gson.JsonElement;

public class MarkerArray extends frc.robot.ros.messages.RosMessage {

    private ArrayList<frc.robot.ros.messages.visualization_msgs.Marker> markers = new ArrayList<>();

    public final String _type = "visualization_msgs/MarkerArray";

    public MarkerArray() {

    }

    public MarkerArray(frc.robot.ros.messages.visualization_msgs.Marker[] markers) {
        this.markers = new ArrayList<>(Arrays.asList(markers));
    }

    public MarkerArray(JsonObject jsonObj) {
        for (JsonElement markers_element : jsonObj.getAsJsonArray("markers")) {
            this.markers.add(new frc.robot.ros.messages.visualization_msgs.Marker(markers_element.getAsJsonObject()));
        }
    }

    public ArrayList<frc.robot.ros.messages.visualization_msgs.Marker> getMarkers() {
        return this.markers;
    }

    public void setMarkers(ArrayList<frc.robot.ros.messages.visualization_msgs.Marker> markers) {
        this.markers = markers;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
