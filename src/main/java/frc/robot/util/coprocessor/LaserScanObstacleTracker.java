// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;

public class LaserScanObstacleTracker {
    private ArrayList<PointObstacle> obstacles;
    private BoundingBox bbox;
    private final double kReverseFanRadians = Units.degreesToRadians(120.0);  // Range of angles to accept as a valid reverse command

    public LaserScanObstacleTracker() {
        
    }

    public void setBoundingBox(BoundingBox bbox) {
        this.bbox = bbox;
    }

    public int size() {
        return obstacles.size();
    }

    public void set(double[] laserXs, double[] laserYs) {
        if (laserXs.length != laserYs.length) {
            System.out.println(String.format(
                "Warning! Laser obstacle coordinate lengths do not match. %d != %d", 
                laserXs.length, laserYs.length
            ));
        }

        int length = Math.min(laserXs.length, laserYs.length);
        while (obstacles.size() < length) {
            obstacles.add(new PointObstacle());
        }

        for (int index = 0; index < length; index++) {
            obstacles.get(index).set(laserXs[index], laserYs[index]);
        }
    }

    public boolean isObstacleWithinBounds()
    {
        for (PointObstacle obstacle : obstacles) {
            if (bbox.isObstacleWithinBounds(obstacle)) {
                return true;
            }
        }
        return false;
    }

    public Pair<Double, Double> getAllowableReverseDirections()
    {
        double minAngle = Double.NaN;
        double maxAngle = Double.NaN;
        for (PointObstacle obstacle : obstacles) {
            if (bbox.isObstacleWithinBounds(obstacle)) {
                double angle = Helpers.boundHalfAngle(Math.atan2(obstacle.getY(), obstacle.getX()) + Math.PI);
                double minObsAngle = angle - kReverseFanRadians / 2.0;
                double maxObsAngle = angle + kReverseFanRadians / 2.0;
                if (Double.isNaN(minAngle) || minObsAngle > minAngle) {
                    minAngle = minObsAngle;
                }
                if (Double.isNaN(maxAngle) || maxObsAngle < maxAngle) {
                    maxAngle = maxObsAngle;
                }
            }
        }

        return new Pair<Double, Double>(minAngle, maxAngle);
    }
}
