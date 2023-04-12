package frc.robot.ros.messages;

import com.google.gson.JsonObject;

public class Time extends frc.robot.ros.messages.RosMessage {

    private int secs = 0;
    private int nsecs = 0;

    public final String _type = "std_msgs/Time";

    public Time() {

    }

    public Time(int secs, int nsecs) {
        this.secs = secs;
        this.nsecs = nsecs;
    }

    public Time(double seconds) {
        this.secs = (int) seconds;
        this.nsecs = (int) (seconds * 1e9);
    }

    public Time(JsonObject jsonObj) {
        this.secs = jsonObj.get("secs").getAsInt();
        this.nsecs = jsonObj.get("nsecs").getAsInt();
    }

    public Time(Duration duration) {
        this.secs = duration.getSecs();
        this.nsecs = duration.getNsecs();
    }

    public int getSecs() {
        return this.secs;
    }

    public int getNsecs() {
        return this.nsecs;
    }

    public void setSecs(int secs) {
        this.secs = secs;
    }

    public void setNsecs(int nsecs) {
        this.nsecs = nsecs;
    }

    public double toSeconds() {
        return (double) secs + (1e-9 * nsecs);
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }

    public Duration minus(Time other) {
        return new Duration(this.secs - other.getSecs(), this.nsecs - other.getNsecs());
    }

    public Time plus(Duration other) {
        return new Time(this.secs + other.getSecs(), this.nsecs + other.getNsecs());
    }
}
