package frc.robot.ros.bridge;

import java.lang.reflect.InvocationTargetException;
import java.util.Base64;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import frc.robot.ros.messages.RosMessage;

public class BridgeSubscriber<T extends RosMessage> {
    private ROSNetworkTablesBridge bridge;
    private String topicName;
    private Class<T> reference;
    private StringSubscriber sub = null;
    private long prevAtomic = 0;
    private T cached = null;

    public BridgeSubscriber(ROSNetworkTablesBridge bridge, String topicName, Class<T> reference) {
        this.bridge = bridge;
        this.topicName = topicName;
        this.reference = reference;
    }

    public boolean didUpdate() {
        if (sub == null) {
            this.sub = this.bridge.subscribe(this.topicName);
        }
        TimestampedString stampedString = sub.getAtomic();
        return stampedString.timestamp != prevAtomic || cached == null;
    }

    public T receive() {
        if (didUpdate()) {
            TimestampedString stampedString = sub.getAtomic();
            String encodedString = stampedString.value;
            byte[] decodedBytes = Base64.getDecoder().decode(encodedString);
            String decodedString = new String(decodedBytes);
            JsonObject jsonObject = JsonParser.parseString(decodedString).getAsJsonObject();
            try {
                cached = reference.getConstructor(JsonObject.class).newInstance(jsonObject);
            } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                    | InvocationTargetException | NoSuchMethodException | SecurityException e) {
                e.printStackTrace();
                throw new RuntimeException("Failed to instantiate ros message");
            }
        }
        return cached;
    }
}
