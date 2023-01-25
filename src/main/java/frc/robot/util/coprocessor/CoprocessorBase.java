// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.coprocessor.detections.Detection;
import frc.robot.util.coprocessor.detections.DetectionManager;

public class CoprocessorBase {
    protected final long DEFAULT_MESSAGE_TIMEOUT = 1_000_000;

    protected final ChassisInterface chassis;

    protected VelocityCommand command = new VelocityCommand();
    protected MessageTimer commandTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    
    protected Pose2d globalPose = new Pose2d();
    protected MessageTimer globalPoseTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);

    protected int numSentGoals = 0;

    protected ArrayList<Double> jointStates = new ArrayList<>();
    protected ArrayList<Double> jointCommands = new ArrayList<>();
    protected MessageTimer jointCommandTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);

    protected Map<String, Pose2d> waypoints = new HashMap<>();

    protected LaserScanObstacleTracker laserObstacles = new LaserScanObstacleTracker();
    protected ZoneManager zoneManager = new ZoneManager();
    protected DetectionManager detectionManager = new DetectionManager();

    public CoprocessorBase(ChassisInterface chassis) {
        this.chassis = chassis;
        laserObstacles.setBoundingBox(chassis.getBoundingBox());
    }

    public void update() {
        
    }

    public boolean isConnected() {
        return true;
    }

    /***
     * Getters for data received from coprocessor
     */

    public boolean isCommandActive() {
        return commandTimer.isActive();
    }

    public VelocityCommand getCommand() {
        return command;
    }

    public Pose2d getGlobalPose() {
        return globalPose;
    }

    public boolean isGlobalPoseActive() {
        return globalPoseTimer.isActive();
    }

    public double getJointCommand(int jointIndex) {
        return jointCommands.get(jointIndex);
    }

    public boolean isJointCommandActive(int jointIndex) {
        return jointCommandTimer.isActive();
    }

    public Pose2d getWaypoint(String waypointName) {
        waypointName = Helpers.parseName(waypointName);
        if (doesWaypointExist(waypointName)) {
            return waypoints.get(waypointName);
        }
        else {
            return new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
        }
    }

    public Pose2d getPoseRelativeToWaypoint(String waypointName, Pose2d relativePose) {
        Pose2d waypoint = getWaypoint(waypointName);
        if (!isPoseValid(waypoint)) {
            return waypoint;
        }
        
        return waypoint.transformBy(new Transform2d(relativePose, new Pose2d()));
    }

    public boolean isPoseValid(Pose2d pose) {
        return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians());
    }

    protected void putWaypoint(String waypointName, Pose2d pose) {
        waypoints.put(waypointName, pose);
    }

    public boolean doesWaypointExist(String waypointName) {
        return waypoints.containsKey(waypointName);
    }

    public Set<String> getWaypointNames() {
        return waypoints.keySet();
    }

    protected double getTime() {
        return RobotController.getFPGATime() * 1E-6;
    }

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        // String waypointName = WaypointMap.parseWaypointName(waypoint.waypoint_name);
        numSentGoals++;
    }

    public void executeGoal() {
        numSentGoals = 0;
    }

    public void cancelGoal() {
        
    }
    
    public void resetPlan() {
        numSentGoals = 0;
    }

    protected void sendMatchStatus(boolean is_autonomous, double match_timer, DriverStation.Alliance team_color) {
        
    }

    public void sendPoseEstimate(Pose2d poseEstimation) {
        
    }

    public void sendImu(double roll, double pitch, double yaw, double angular_z, double accel_x, double accel_y) {

    }

    public void setJointPosition(int index, double position) {
        while (index >= jointStates.size()) {
            jointStates.add(0.0);
            jointCommands.add(0.0);
        }
        jointStates.set(index, position);
    }

    public String parseObjectName(String objectName) {
        return Helpers.parseName(objectName);
    }

    public LaserScanObstacleTracker getLaserScanObstacles() {
        return laserObstacles;
    }

    public boolean areZonesValid() {
        return zoneManager.isValid();
    }

    public ZoneInfo getNearestNoGoZone() {
        return zoneManager.getNearestNoGoZone();
    }

    public ZoneInfo getNearestZone() {
        return zoneManager.getNearestZone();
    }

    public ZoneManager getZoneManager() {
        return zoneManager;
    }

    public void setNoGoZones(String[] names) {
        zoneManager.setNoGoes(names);
    }

    public void setNoGoZone(String name) {
        zoneManager.setNoGo(name);
    }

    public void removeNoGoZone(String name) {
        zoneManager.removeNoGo(name);
    }

    public boolean doesDetectionNameExist(String name) {
        return detectionManager.doesNameExist(name);
    }
    public boolean doesDetectionExist(String name, int index) {
        return detectionManager.doesDetectionExist(name, index);
    }

    public Detection getDetection(String name, int index) {
        return detectionManager.getDetection(name, index);
    }

    public Collection<Detection> getAllDetectionsNamed(String name) {
        return detectionManager.getAllDetectionsNamed(name);
    }

    public Collection<Detection> getAllDetections() {
        return detectionManager.getAllDetections();
    }

    /**
     * @param heading direction of travel of the robot from the robot's perspective
     * @param reverseFanRadians range of accepted angles for each extended beam
     * @param distanceRangeMeters if no go zone is further than this distance, ignore it
     * @return
     */
    public boolean isDirectionTowardNoGoZonesAllowed(double heading, double reverseFanRadians, double distanceRangeMeters) {
        Set<String> names = zoneManager.getNoGoNames();
        List<String> in_bound_names = new ArrayList<>(names);
        in_bound_names.removeIf(name -> zoneManager.getZone(name).getDistance() > distanceRangeMeters);
        double angles[] = new double[in_bound_names.size()];

        int index = 0;
        Pose2d robot_pose = getGlobalPose();
        for (String name : in_bound_names) {
            ZoneInfo zone = zoneManager.getZone(name);
            Pose2d nearest_pose = new Pose2d(zone.getNearestX(), zone.getNearestY(), new Rotation2d());
            Pose2d relative_nearest = nearest_pose.relativeTo(robot_pose);
            angles[index++] = Math.atan2(
                relative_nearest.getY(),
                relative_nearest.getX()
            );
        }
        return Helpers.isDirectionAllowed(heading, angles, reverseFanRadians);
    }
}
