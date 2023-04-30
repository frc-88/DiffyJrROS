package frc.robot.trajectory.custompathweaver;

public class PathweaverTrajectoryElement {
    public double time = 0.0;
    public PathweaverTrajectoryPose pose = new PathweaverTrajectoryPose();
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double curvature = 0.0;
    public double holonomicRotation = 0.0;
    public double angularVelocity = 0.0;
    public double holonomicAngularVelocity = 0.0;
}
