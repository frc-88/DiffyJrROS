// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive_subsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.diffswerve.DiffSwerveChassis;
import frc.robot.diffswerve.NavX;

public class DriveSubsystem extends SubsystemBase {
    private DiffSwerveChassis swerve;
    private NavX imu;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        imu = new NavX();
        swerve = new DiffSwerveChassis(imu);
    }

    @Override
    public void periodic() {
        swerve.periodic();
    }

    public DiffSwerveChassis getSwerve() {
        return swerve;
    }

    public NavX getImu() {
        return imu;
    }

    public void setCoast(boolean coast) {
        swerve.setCoast(coast);
    }

    public void setEnabled(boolean enabled) {
        swerve.setEnabled(enabled);
    }

    public void drive(ChassisSpeeds command) {
        swerve.drive(command);
    }

    public void stop() {
        swerve.stop();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
