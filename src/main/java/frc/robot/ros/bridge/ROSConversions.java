package frc.robot.ros.bridge;

public class ROSConversions {
    public static frc.robot.ros.messages.geometry_msgs.Quaternion wpiToRosRotation(
            edu.wpi.first.math.geometry.Rotation3d rotate) {
        edu.wpi.first.math.geometry.Quaternion quat = rotate.getQuaternion();
        return new frc.robot.ros.messages.geometry_msgs.Quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    public static edu.wpi.first.math.geometry.Rotation3d rosToWpiRotation(
            frc.robot.ros.messages.geometry_msgs.Quaternion quat) {
        return new edu.wpi.first.math.geometry.Rotation3d(
                new edu.wpi.first.math.geometry.Quaternion(quat.getW(), quat.getX(), quat.getY(), quat.getZ()));
    }

    public static frc.robot.ros.messages.geometry_msgs.Point wpiToRosTranslation(
            edu.wpi.first.math.geometry.Translation3d point) {
        return new frc.robot.ros.messages.geometry_msgs.Point(point.getX(), point.getY(), point.getZ());
    }

    public static edu.wpi.first.math.geometry.Translation3d rosToWpiTranslation(
            frc.robot.ros.messages.geometry_msgs.Point point) {
        return new edu.wpi.first.math.geometry.Translation3d(point.getX(), point.getY(), point.getZ());
    }

    public static frc.robot.ros.messages.geometry_msgs.Pose wpiToRosPose(edu.wpi.first.math.geometry.Pose3d pose) {
        return new frc.robot.ros.messages.geometry_msgs.Pose(wpiToRosTranslation(pose.getTranslation()),
                wpiToRosRotation(pose.getRotation()));
    }

    public static edu.wpi.first.math.geometry.Pose3d rosToWpiPose(frc.robot.ros.messages.geometry_msgs.Pose pose) {
        return new edu.wpi.first.math.geometry.Pose3d(rosToWpiTranslation(pose.getPosition()),
                rosToWpiRotation(pose.getOrientation()));
    }

    public static edu.wpi.first.math.kinematics.ChassisSpeeds rosToWpiTwist(
            frc.robot.ros.messages.geometry_msgs.Twist twist) {
        return new edu.wpi.first.math.kinematics.ChassisSpeeds(
                twist.getLinear().getX(),
                twist.getLinear().getY(),
                twist.getAngular().getZ());
    }

    public static frc.robot.ros.messages.geometry_msgs.Twist wpiToRosTwist(
            edu.wpi.first.math.kinematics.ChassisSpeeds twist) {
        return new frc.robot.ros.messages.geometry_msgs.Twist(
                new frc.robot.ros.messages.geometry_msgs.Vector3(twist.vxMetersPerSecond, twist.vyMetersPerSecond, 0.0),
                new frc.robot.ros.messages.geometry_msgs.Vector3(0.0, 0.0, twist.omegaRadiansPerSecond));
    }
}
