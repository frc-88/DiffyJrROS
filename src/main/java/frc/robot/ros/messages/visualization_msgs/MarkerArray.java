// Auto generated!! Do not modify.
package frc.robot.ros.messages.visualization_msgs;

import com.google.gson.JsonObject;
import com.google.gson.Gson;
import java.util.Arrays;
import com.google.gson.JsonElement;
import java.util.ArrayList;

public class MarkerArray implements frc.robot.ros.messages.RosMessage {

    private ArrayList<frc.robot.ros.messages.visualization_msgs.Marker> markers = new ArrayList<>();

    Gson ginst = new Gson();

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
