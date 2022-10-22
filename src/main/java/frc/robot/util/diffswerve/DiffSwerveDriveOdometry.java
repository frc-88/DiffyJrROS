// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.diffswerve;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your swerve drive encoders and swerve azimuth
 * encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 */
public class DiffSwerveDriveOdometry {
  private final SwerveDriveKinematics m_kinematics;
  private Pose2d m_poseMeters;
  private double m_prevTimeSeconds = -1;

  /**
   * Constructs a DiffSwerveDriveOdometry object.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  public DiffSwerveDriveOdometry(
      SwerveDriveKinematics kinematics, Pose2d initialPose) {
    m_kinematics = kinematics;
    m_poseMeters = initialPose;
    MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
  }

  /**
   * Constructs a DiffSwerveDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   */
  public DiffSwerveDriveOdometry(SwerveDriveKinematics kinematics) {
    this(kinematics, new Pose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param pose The position on the field that your robot is at.
   */
  public void resetPosition(Pose2d pose) {
    m_poseMeters = pose;
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in the current time as a parameter to calculate period (difference
   * between two timestamps). The period is used to calculate the change in distance from a
   * velocity. This also takes in an angle parameter which is used instead of the angular rate that
   * is calculated from forward kinematics.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param moduleStates The current state of all swerve modules. Please provide the states in the
   *     same order in which you instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, SwerveModuleState... moduleStates) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
    m_poseMeters =
        m_poseMeters.exp(
            new Twist2d(
                chassisState.vxMetersPerSecond * period,
                chassisState.vyMetersPerSecond * period,
                chassisState.omegaRadiansPerSecond * period
              ));

    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method automatically calculates the current time to calculate period
   * (difference between two timestamps). The period is used to calculate the change in distance
   * from a velocity. This also takes in an angle parameter which is used instead of the angular
   * rate that is calculated from forward kinematics.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param moduleStates The current state of all swerve modules. Please provide the states in the
   *     same order in which you instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public Pose2d update(SwerveModuleState... moduleStates) {
    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, moduleStates);
  }
}
