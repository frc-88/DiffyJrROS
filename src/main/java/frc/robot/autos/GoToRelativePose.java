package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.localization.Localization;
import frc.robot.trajectory.CustomTrajectoryGenerator;
import frc.robot.trajectory.Waypoint;

public class GoToRelativePose extends FollowTrajectoryBase {
    private final Pose2d relativeGoalPose;

    public GoToRelativePose(DriveSubsystem drive, Localization localization, Pose2d relativeGoalPose) {
        super(drive, localization);
        this.relativeGoalPose = relativeGoalPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!isPoseValid()) {
            setHasFailed(true);
            return;
        }
        Pose2d currentPose = getPose();

        Transform2d transform = new Transform2d(new Pose2d(), currentPose);
        Pose2d goalPose = relativeGoalPose.transformBy(transform);

        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(currentPose.getTranslation()));
        waypoints.add(new Waypoint(goalPose.getTranslation()));

        TrajectoryConfig config = makeTrajectoryConfig(0.0, List.of());
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();
        generator.generate(config, waypoints);
        setTrajectoryFromGenerator(generator);
        super.initialize();
    }
}
