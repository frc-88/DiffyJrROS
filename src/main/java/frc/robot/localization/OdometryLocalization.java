package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;

public class OdometryLocalization implements Localization {
    private final DriveSubsystem drive;

    public OdometryLocalization(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public Pose2d getPose() {
        return this.drive.getSwerve().getOdometryPose();
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return this.drive.getSwerve().getChassisSpeeds();
    }

    @Override
    public boolean isValid() {
        return true;
    }
}
