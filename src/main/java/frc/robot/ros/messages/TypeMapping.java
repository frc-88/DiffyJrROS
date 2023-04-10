// Auto generated!! Do not modify.
package frc.robot.ros.messages;

import java.util.Map;
import java.util.HashMap;
import java.lang.reflect.InvocationTargetException;
import com.google.gson.JsonObject;

public class TypeMapping {
    public static final Map<String, Class<? extends RosMessage>> MAPPING = new HashMap<>();

    static {
        MAPPING.put("tj2_interfaces/GameObjectsStamped", frc.robot.ros.messages.tj2_interfaces.GameObjectsStamped.class);
        MAPPING.put("std_msgs/Header", frc.robot.ros.messages.std_msgs.Header.class);
        MAPPING.put("tj2_interfaces/GameObject", frc.robot.ros.messages.tj2_interfaces.GameObject.class);
        MAPPING.put("geometry_msgs/Pose", frc.robot.ros.messages.geometry_msgs.Pose.class);
        MAPPING.put("geometry_msgs/Point", frc.robot.ros.messages.geometry_msgs.Point.class);
        MAPPING.put("geometry_msgs/Quaternion", frc.robot.ros.messages.geometry_msgs.Quaternion.class);
        MAPPING.put("tj2_interfaces/UVBoundingBox", frc.robot.ros.messages.tj2_interfaces.UVBoundingBox.class);
        MAPPING.put("tj2_interfaces/UVKeypoint", frc.robot.ros.messages.tj2_interfaces.UVKeypoint.class);
        MAPPING.put("tj2_interfaces/XYZBoundingBox", frc.robot.ros.messages.tj2_interfaces.XYZBoundingBox.class);
        MAPPING.put("tj2_interfaces/XYZKeypoint", frc.robot.ros.messages.tj2_interfaces.XYZKeypoint.class);
        MAPPING.put("geometry_msgs/Vector3", frc.robot.ros.messages.geometry_msgs.Vector3.class);
        MAPPING.put("visualization_msgs/MarkerArray", frc.robot.ros.messages.visualization_msgs.MarkerArray.class);
        MAPPING.put("visualization_msgs/Marker", frc.robot.ros.messages.visualization_msgs.Marker.class);
        MAPPING.put("std_msgs/ColorRGBA", frc.robot.ros.messages.std_msgs.ColorRGBA.class);
        MAPPING.put("std_msgs/Float32MultiArray", frc.robot.ros.messages.std_msgs.Float32MultiArray.class);
        MAPPING.put("std_msgs/MultiArrayLayout", frc.robot.ros.messages.std_msgs.MultiArrayLayout.class);
        MAPPING.put("std_msgs/MultiArrayDimension", frc.robot.ros.messages.std_msgs.MultiArrayDimension.class);
        MAPPING.put("tj2_interfaces/TestStaticArray", frc.robot.ros.messages.tj2_interfaces.TestStaticArray.class);

    }

    public RosMessage fromJSON(JsonObject jsonObj) {
        try {
            return MAPPING.get(jsonObj.get("_type").getAsString()).getConstructor(JsonObject.class)
                    .newInstance(jsonObj);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
                | NoSuchMethodException | SecurityException e) {
            e.printStackTrace();
            return null;
        }
    }
}
