// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.annotations.Expose;
import com.google.gson.JsonObject;

public class Header extends frc.robot.ros.messages.RosMessage {

    private int seq = 0;
    private frc.robot.ros.messages.Time stamp = new frc.robot.ros.messages.Time();
    private String frame_id = "";

    @Expose(serialize = false, deserialize = false)
    public final String _type = "std_msgs/Header";

    public Header() {

    }

    public Header(int seq, frc.robot.ros.messages.Time stamp, String frame_id) {
        this.seq = seq;
        this.stamp = stamp;
        this.frame_id = frame_id;
    }

    public Header(JsonObject jsonObj) {
        this.seq = jsonObj.get("seq").getAsInt();
        this.stamp = new frc.robot.ros.messages.Time(jsonObj.get("stamp").getAsJsonObject());
        this.frame_id = jsonObj.get("frame_id").getAsString();
    }

    public int getSeq() {
        return this.seq;
    }
    public frc.robot.ros.messages.Time getStamp() {
        return this.stamp;
    }
    public String getFrameId() {
        return this.frame_id;
    }

    public void setSeq(int seq) {
        this.seq = seq;
    }
    public void setStamp(frc.robot.ros.messages.Time stamp) {
        this.stamp = stamp;
    }
    public void setFrameId(String frame_id) {
        this.frame_id = frame_id;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
