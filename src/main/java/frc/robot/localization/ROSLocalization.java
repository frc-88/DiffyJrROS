package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DiffyJrCoprocessorBridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;

public class ROSLocalization implements Localization {
    private final DiffyJrCoprocessorBridge bridge;
    private final DriveSubsystem drive;
    private final TFListenerCompact tf_compact;

    public ROSLocalization(DriveSubsystem drive, DiffyJrCoprocessorBridge bridge) {
        this.bridge = bridge;
        this.drive = drive;
        this.tf_compact = this.bridge.getTFListener();
    }

    @Override
    public Pose2d getPose() {
        if (tf_compact.canTransform(DiffyJrCoprocessorBridge.MAP_FRAME, DiffyJrCoprocessorBridge.BASE_FRAME)) {
            Transform3dStamped tfStamped = tf_compact.lookupTransform(DiffyJrCoprocessorBridge.MAP_FRAME,
                    DiffyJrCoprocessorBridge.BASE_FRAME);
            return new Pose2d(
                    tfStamped.transform.getTranslation().toTranslation2d(),
                    tfStamped.transform.getRotation().toRotation2d());

        } else {
            DriverStation.reportError("Failed to get transform from ROS!!", false);
            return new Pose2d();
        }
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return this.drive.getSwerve().getChassisSpeeds();
    }
}
