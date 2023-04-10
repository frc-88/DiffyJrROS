// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonObject;

public class UVKeypoint implements frc.robot.ros.messages.RosMessage {
    private int x = 0;
    private int y = 0;


    public UVKeypoint() {

    }

    public UVKeypoint(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public UVKeypoint(JsonObject jsonObj) {
        this.x = jsonObj.get("x").getAsInt();
        this.y = jsonObj.get("y").getAsInt();
    }

    public int getX() {
        return this.x;
    }

    public int getY() {
        return this.y;
    }



    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }



    public String toString() {

        return "";
    }

    
}
