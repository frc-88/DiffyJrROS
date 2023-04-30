package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Localization {
    public boolean isValid();

    public Pose2d getPose();

    public ChassisSpeeds getVelocity();
}
