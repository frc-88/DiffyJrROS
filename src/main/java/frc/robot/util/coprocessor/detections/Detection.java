package frc.robot.util.coprocessor.detections;

public class Detection {
    // All poses are assumed to be in the base_link frame
    // (center of the robot projected onto the ground)

    private String name;
    private int index;
    private Pose3d pose = new Pose3d();

    public Detection(String name, int index, Pose3d pose) {
        this.name = name;
        this.index = index;
        this.pose = pose;
    }
    
    public Detection(String name, int index, Position position, Orientation orientation) {
        this.name = name;
        this.index = index;
        this.pose.position = position;
        this.pose.orientation = orientation;
    }

    public Detection(
        String name,
        int index,
        double position_x,
        double position_y,
        double position_z,
        double orientation_w,
        double orientation_x,
        double orientation_y,
        double orientation_z
    ) {
        this.name = name;
        this.index = index;
        pose.position.x = position_x;
        pose.position.y = position_y;
        pose.position.z = position_z;
        pose.orientation.w = orientation_w;
        pose.orientation.x = orientation_x;
        pose.orientation.y = orientation_y;
        pose.orientation.z = orientation_z;
    }

    public String getName() {
        return name;
    }

    public int getIndex() {
        return index;
    }


    public Position getPosition() {
        return pose.position;
    }

    public Orientation getOrientation() {
        return pose.orientation;
    }

    public Pose3d getPose() {
        return pose;
    }
}
