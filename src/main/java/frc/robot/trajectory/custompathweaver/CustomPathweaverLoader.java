package frc.robot.trajectory.custompathweaver;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.trajectory.RotationSequence;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.TreeMap;

import com.google.gson.Gson;

public class CustomPathweaverLoader {
    private final static Gson ginst = new Gson();

    public static Pair<Trajectory, RotationSequence> fromPathweaverJson(Path path) throws IOException {
        String jsonString = Files.readString(path);
        PathweaverTrajectoryJSON response = ginst.fromJson(jsonString, PathweaverTrajectoryJSON.class);
        Trajectory trajectory = new Trajectory();
        TreeMap<Double, Rotation2d> rotations = new TreeMap<>();
        for (PathweaverTrajectoryElement element : response.trajectory) {
            trajectory.getStates()
                    .add(new State(
                            element.time,
                            element.velocity,
                            element.acceleration,
                            new Pose2d(
                                    element.pose.translation.x,
                                    element.pose.translation.y,
                                    new Rotation2d(element.pose.rotation.radians)),
                            element.curvature));
            rotations.put(element.time, Rotation2d.fromDegrees(element.holonomicRotation));
            // TODO: add holonomic angular velocities:
            // new RotationSequence.State(Rotation2d.fromDegrees(element.holonomicRotation),
            // element.holonomicAngularVelocity)
        }
        RotationSequence rotationSequence = new RotationSequence(rotations);
        return new Pair<Trajectory, RotationSequence>(trajectory, rotationSequence);
    }
}
