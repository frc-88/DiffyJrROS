// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;

public class MatchPeriod extends frc.robot.ros.messages.RosMessage {
    public static int TELEOP = 2;
    public static int AUTONOMOUS = 1;
    public static int DISABLED = 0;

    private int type = 0;

    public final String _type = "tj2_interfaces/MatchPeriod";

    public MatchPeriod() {

    }

    public MatchPeriod(int type) {
        this.type = type;
    }

    public MatchPeriod(JsonObject jsonObj) {
        this.type = jsonObj.get("type").getAsInt();
    }

    public int getType() {
        return this.type;
    }

    public void setType(int type) {
        this.type = type;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
