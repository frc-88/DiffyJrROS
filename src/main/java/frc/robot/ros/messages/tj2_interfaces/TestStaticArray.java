// Auto generated!! Do not modify.
package frc.robot.ros.messages.tj2_interfaces;

import com.google.gson.JsonElement;
import com.google.gson.Gson;
import com.google.gson.JsonObject;

public class TestStaticArray implements frc.robot.ros.messages.RosMessage {

    private Float[] test1 = new Float[] {
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };
    private String[] test2 = new String[] {
        "",
        "",
        "",
        "",
        ""
    };

    Gson ginst = new Gson();

    public TestStaticArray() {

    }

    public TestStaticArray(Float[] test1, String[] test2) {
        for (int index = 0; index < 4; index++) {
            this.test1[index] = test1[index];
        }
        for (int index = 0; index < 5; index++) {
            this.test2[index] = test2[index];
        }
    }

    public TestStaticArray(JsonObject jsonObj) {
        int test1_element_index = 0;
        for (JsonElement test1_element : jsonObj.getAsJsonArray("test1")) {
            this.test1[test1_element_index] = test1_element.getAsFloat();
        }
        int test2_element_index = 0;
        for (JsonElement test2_element : jsonObj.getAsJsonArray("test2")) {
            this.test2[test2_element_index] = test2_element.getAsString();
        }
    }

    public Float[] getTest1() {
        return this.test1;
    }
    public String[] getTest2() {
        return this.test2;
    }

    public void setTest1(Float[] test1) {
        this.test1 = test1;
    }
    public void setTest2(String[] test2) {
        this.test2 = test2;
    }

    public JsonObject toJSON() {
        return ginst.toJsonTree(this).getAsJsonObject();
    }

    public String toString() {
        return ginst.toJson(this);
    }
}
