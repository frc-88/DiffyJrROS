package frc.robot.util.coprocessor.detections;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class DetectionManager {
    private Map<String, Map<Integer, Detection>> detections;
    
    public DetectionManager() {
        detections = new HashMap<>();
    }

    public void setDetection(String name, int index, Detection detection) {
        if (detections.containsKey(name)) {
            detections.get(name).put(index, detection);
        }
        else {
            detections.put(name, new HashMap<>());
        }
    }

    public boolean doesNameExist(String name) {
        return detections.containsKey(name);
    }

    public boolean doesDetectionExist(String name, int index) {
        if (doesNameExist(name)) {
            return detections.get(name).containsKey(index);
        }
        else {
            return false;
        }
    }

    public Detection getDetection(String name, int index) {
        return detections.get(name).get(index);
    }

    public Collection<Detection> getAllDetectionsNamed(String name) {
        return detections.get(name).values();
    }

    public Collection<Detection> getAllDetections() {
        Collection<Detection> all = new ArrayList<Detection>();
        for (String name : detections.keySet()) {
            for (Detection detection : detections.get(name).values()) {
                all.add(detection);
            }
        }
        return all;
    }
}
