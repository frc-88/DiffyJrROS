// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;
import com.google.gson.annotations.Expose;

public class RequestFrames extends frc.team88.ros.messages.RosMessage {

    private frc.team88.ros.messages.std_msgs.RosHeader header = new frc.team88.ros.messages.std_msgs.RosHeader();
    private frc.team88.ros.messages.DurationPrimitive request_duration = new frc.team88.ros.messages.DurationPrimitive();
    private int num_frames = 0;

    @Expose(serialize = false, deserialize = false)
    public final java.lang.String _type = "tj2_interfaces/RequestFrames";

    public RequestFrames() {

    }

    public RequestFrames(frc.team88.ros.messages.std_msgs.RosHeader header, frc.team88.ros.messages.DurationPrimitive request_duration, int num_frames) {
        this.header = header;
        this.request_duration = request_duration;
        this.num_frames = num_frames;
    }

    public RequestFrames(JsonObject jsonObj) {
        this.header = new frc.team88.ros.messages.std_msgs.RosHeader(jsonObj.get("header").getAsJsonObject());
        this.request_duration = new frc.team88.ros.messages.DurationPrimitive(jsonObj.get("request_duration").getAsJsonObject());
        this.num_frames = jsonObj.get("num_frames").getAsInt();
    }

    public frc.team88.ros.messages.std_msgs.RosHeader getHeader() {
        return this.header;
    }
    public frc.team88.ros.messages.DurationPrimitive getRequestDuration() {
        return this.request_duration;
    }
    public int getNumFrames() {
        return this.num_frames;
    }

    public void setHeader(frc.team88.ros.messages.std_msgs.RosHeader header) {
        this.header = header;
    }
    public void setRequestDuration(frc.team88.ros.messages.DurationPrimitive request_duration) {
        this.request_duration = request_duration;
    }
    public void setNumFrames(int num_frames) {
        this.num_frames = num_frames;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public java.lang.String toString() {
        return ginst.toJson(this);
    }
}
