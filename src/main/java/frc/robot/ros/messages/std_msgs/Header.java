// Auto generated!! Do not modify.
package frc.robot.ros.messages.std_msgs;

import com.google.gson.JsonObject;

public class Header implements frc.robot.ros.messages.RosMessage {
    private int seq = 0;
    private long stamp = 0;
    private String frame_id = "";


    public Header() {

    }

    public Header(int seq, long stamp, String frame_id) {
        this.seq = seq;
        this.stamp = stamp;
        this.frame_id = frame_id;
    }

    public Header(JsonObject jsonObj) {
        this.seq = jsonObj.get("seq").getAsInt();
        this.stamp = jsonObj.get("stamp").getAsLong();
        this.frame_id = jsonObj.get("frame_id").getAsString();
    }

    public int getSeq() {
        return this.seq;
    }

    public long getStamp() {
        return this.stamp;
    }

    public String getFrameId() {
        return this.frame_id;
    }



    public void setSeq(int seq) {
        this.seq = seq;
    }

    public void setStamp(long stamp) {
        this.stamp = stamp;
    }

    public void setFrameId(String frame_id) {
        this.frame_id = frame_id;
    }



    public String toString() {

        return "";
    }

    
}
