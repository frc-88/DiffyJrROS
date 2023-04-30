package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;

public class OdometryLocalization implements Localization {
    private final DriveSubsystem drive;
    private Pose2d odomOrigin = new Pose2d();
    private Pose2d absoluteOrigin = new Pose2d();
    private boolean isOriginSet = false;

    public OdometryLocalization(DriveSubsystem drive) {
        this.drive = drive;
    }

    private Pose2d getOdomPose() {
        return this.drive.getSwerve().getOdometryPose();
    }

    @Override
    public Pose2d getPose() {
        Pose2d odomPose = getOdomPose();
        if (isOriginSet) {
            Pose2d relativePose = odomPose.relativeTo(odomOrigin);
            Pose2d absolutePose = relativePose.transformBy(new Transform2d(new Pose2d(), absoluteOrigin));
            return absolutePose;
        } else {
            return odomPose;
        }
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return this.drive.getSwerve().getChassisSpeeds();
    }

    @Override
    public boolean isValid() {
        return isOriginSet;
    }

    @Override
    public void reset(Pose2d pose) {
        odomOrigin = getOdomPose();
        absoluteOrigin = pose;
        isOriginSet = true;
    }
}
