package frc.robot.ros.messages;

import com.google.gson.JsonObject;

public class Time extends frc.robot.ros.messages.RosMessage {

    private int sec = 0;
    private int nsec = 0;

    public final String _type = "std_msgs/Time";

    public Time() {

    }

    public Time(int sec, int nsec) {
        this.sec = sec;
        this.nsec = nsec;
    }

    public Time(double seconds) {
        this.sec = (int) seconds;
        this.nsec = (int) (seconds * 1e9);
    }

    public Time(JsonObject jsonObj) {
        this.sec = jsonObj.get("sec").getAsInt();
        this.nsec = jsonObj.get("nsec").getAsInt();
    }

    public Time(Duration duration) {
        this.sec = duration.getSec();
        this.nsec = duration.getNsec();
    }

    public int getSec() {
        return this.sec;
    }

    public int getNsec() {
        return this.nsec;
    }

    public void setSec(int sec) {
        this.sec = sec;
    }

    public void setNsec(int nsec) {
        this.nsec = nsec;
    }

    public double toSeconds() {
        return (double) sec + (1e-9 * nsec);
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }

    public Duration minus(Time other) {
        return new Duration(this.sec - other.getSec(), this.nsec - other.getNsec());
    }

    public Time plus(Duration other) {
        return new Time(this.sec + other.getSec(), this.nsec + other.getNsec());
    }
}
