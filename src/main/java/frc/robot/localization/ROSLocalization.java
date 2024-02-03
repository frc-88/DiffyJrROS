package frc.robot.localization;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive_subsystem.DriveSubsystem;
import frc.robot.ros.bridge.Frames;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;

public class ROSLocalization implements Localization {
    private final DriveSubsystem drive;
    private final TFListenerCompact tf_compact;

    public ROSLocalization(DriveSubsystem drive, TFListenerCompact tf_compact) {
        this.drive = drive;
        this.tf_compact = tf_compact;
    }

    @Override
    public Pose2d getPose() {
        Optional<Transform3dStamped> tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
        if (tfStamped.isEmpty()) {
            return new Pose2d();
        }
        return new Pose2d(
                tfStamped.get().transform.getTranslation().toTranslation2d(),
                tfStamped.get().transform.getRotation().toRotation2d());
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return this.drive.getSwerve().getChassisSpeeds();
    }

    @Override
    public boolean isValid() {
        return tf_compact.canTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
    }

    @Override
    public void reset(Pose2d pose) {

    }
}
