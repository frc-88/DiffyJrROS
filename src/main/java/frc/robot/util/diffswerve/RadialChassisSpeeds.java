// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.diffswerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class RadialChassisSpeeds {
    public double velocityMetersPerSecond = 0.0;
    public double radiusOfCurvatureMeters = 0.0;
    public double directionalAngleRadians = 0.0;
    public double dtSeconds = 1.0;

    public double omegaRadiansPerSecond = 0.0;  // in case velocityMetersPerSecond / radiusOfCurvatureMeters is undefined, store the original omegaRadiansPerSecond

    RadialChassisSpeeds()  {}
    RadialChassisSpeeds(double velocityMetersPerSecond, double radiusOfCurvatureMeters, double directionalAngleRadians, double dtSeconds) {
        this.velocityMetersPerSecond = velocityMetersPerSecond;
        this.radiusOfCurvatureMeters = radiusOfCurvatureMeters;
        this.directionalAngleRadians = directionalAngleRadians;
        this.dtSeconds = dtSeconds;
    }

    public static RadialChassisSpeeds fromChassisSpeeds(ChassisSpeeds chassisSpeeds, double dtSeconds) {
        RadialChassisSpeeds obj = new RadialChassisSpeeds();

        obj.directionalAngleRadians = Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        obj.velocityMetersPerSecond = Math.sqrt(
            chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond + 
            chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond
        );
        obj.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        obj.dtSeconds = dtSeconds;
        double deltaAngle = chassisSpeeds.omegaRadiansPerSecond * obj.dtSeconds;
        double deltaDistance = obj.velocityMetersPerSecond * obj.dtSeconds;
        if (deltaDistance == 0.0) {
            obj.radiusOfCurvatureMeters = 0.0;
        }
        else if (deltaAngle == 0.0) {
            obj.radiusOfCurvatureMeters = Double.POSITIVE_INFINITY;
        }
        else {
            obj.radiusOfCurvatureMeters = deltaDistance / Math.tan(deltaAngle);
        }

        return obj;
    }

    public ChassisSpeeds toChassisSpeeds() {
        ChassisSpeeds obj = new ChassisSpeeds();
        obj.vxMetersPerSecond = this.velocityMetersPerSecond * Math.cos(this.directionalAngleRadians);
        obj.vyMetersPerSecond = this.velocityMetersPerSecond * Math.sin(this.directionalAngleRadians);
        if (this.dtSeconds == 0.0) {
            obj.omegaRadiansPerSecond = 0.0;
        }
        else if (this.velocityMetersPerSecond == 0.0 && this.radiusOfCurvatureMeters == 0.0) {
            // omegaRadiansPerSecond is undefined. Use the original omegaRadiansPerSecond
            obj.omegaRadiansPerSecond = this.omegaRadiansPerSecond;
        }
        else if (this.radiusOfCurvatureMeters == 0.0) {
            obj.omegaRadiansPerSecond = Math.PI * 0.5 / this.dtSeconds;
        }
        else {
            obj.omegaRadiansPerSecond = Math.atan(this.dtSeconds * this.velocityMetersPerSecond / this.radiusOfCurvatureMeters) / this.dtSeconds;
        }
        return obj;
    }

    @Override
    public String toString() {
        return String.format(
            "RadialChassisSpeeds(v: %.2f m/s, theta: %.2f rad, radius: %.2f m, dt: %.2f s)",
                this.velocityMetersPerSecond, this.directionalAngleRadians, this.radiusOfCurvatureMeters,
                this.dtSeconds);
    }
}
