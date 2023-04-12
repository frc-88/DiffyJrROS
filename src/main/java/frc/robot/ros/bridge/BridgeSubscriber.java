package frc.robot.ros.bridge;

import java.lang.reflect.InvocationTargetException;
import java.util.Base64;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import frc.robot.ros.messages.RosMessage;

public class BridgeSubscriber<T extends RosMessage> {
    private ROSNetworkTablesBridge bridge;
    private String topicName;
    private Class<T> reference;
    private StringSubscriber sub = null;
    private StringPublisher pub = null;
    private long prevAtomic = 0;

    public BridgeSubscriber(ROSNetworkTablesBridge bridge, String topicName, Class<T> reference) {
        this.bridge = bridge;
        this.topicName = topicName;
        this.reference = reference;
    }

    public T receive() {
        if (this.sub == null) {
            Pair<StringSubscriber, StringPublisher> pair = this.bridge.subscribe(this.topicName);
            this.sub = pair.getFirst();
            this.pub = pair.getSecond();
            this.pub.set("");
        }
        if (!this.sub.exists()) {
            System.out.println("Creating topic " + this.topicName);
            this.pub.set("");
        }
        TimestampedString stampedString = this.sub.getAtomic();
        if (stampedString.timestamp == prevAtomic) {
            return null;
        }
        prevAtomic = stampedString.timestamp;
        String encodedString = stampedString.value;
        if (encodedString.length() == 0) {
            return null;
        }
        byte[] decodedBytes = Base64.getDecoder().decode(encodedString);
        String decodedString = new String(decodedBytes);
        JsonObject jsonObject;
        try {
            jsonObject = JsonParser.parseString(decodedString).getAsJsonObject();
        } catch (java.lang.IllegalStateException e) {
            e.printStackTrace();
            return null;
        }
        try {
            return reference.getConstructor(JsonObject.class).newInstance(jsonObject);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                | InvocationTargetException | NoSuchMethodException | SecurityException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to instantiate ros message");
        }
    }
}
