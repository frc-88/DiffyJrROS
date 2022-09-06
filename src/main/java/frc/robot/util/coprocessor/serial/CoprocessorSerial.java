// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor.serial;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.CoprocessorBase;
import frc.robot.util.coprocessor.tunnel.DataStreamInterface;
import frc.robot.util.coprocessor.tunnel.Handshake;
import frc.robot.util.coprocessor.tunnel.PacketResult;
import frc.robot.util.coprocessor.tunnel.TunnelInterface;
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointMap;


public class CoprocessorSerial extends CoprocessorBase implements TunnelInterface {
    final SerialStream data_stream;

    public CoprocessorSerial(ChassisInterface chassis) {
        super(chassis);
        data_stream = new SerialStream(this);
    }

    public void packetCallback(DataStreamInterface data_stream, PacketResult result)
    {
        String category = result.getCategory();

        if (category.equals("cmd")) {
            command = parseVelocityCommand(result);
            commandTimer.reset();
        }
        else if (category.equals("global")) {
            globalPose = parsePose2d(result);
            globalPoseTimer.reset();
        }
        else if (category.equals("goal_status")) {
            Pair<Integer, Boolean> status = result.getInt(); if (!status.getSecond()) { System.out.println("Failed to get goal status"); return; }
            goalStatus = GoalStatus.getStatus(status.getFirst());
            goalStatusTimer.reset();
        }
        else if (category.equals("reset_odom")) {
            Pose2d reset_pose = parsePose2d(result);
            this.chassis.resetPosition(reset_pose);
        }
        else if (category.equals("joint")) {
            parseJoint(result);
        }
        else if (category.equals("waypoint")) {
            Pair<String, Boolean> waypoint_name = result.getString(); if (!waypoint_name.getSecond()) { System.out.println("Failed to get waypoint name"); return; }
            Pose2d waypoint_pose = parsePose2d(result);
            waypoints.put(waypoint_name.getFirst(), waypoint_pose);
        }
        else if (category.equals("ping")) {
            Pair<Double, Boolean> ping = result.getDouble(); if (!ping.getSecond()) { System.out.println("Failed to get ping"); return; }
            data_stream.writePacket("ping", ping.getFirst());
        }
    }

    public void handshakeCallback(DataStreamInterface data_stream, Handshake handshake) {
        System.out.println(String.format("Handshake received. category: %s #%d", handshake.getCategory(), handshake.getPacketNum()));

    }

    private VelocityCommand parseVelocityCommand(PacketResult result) {
        VelocityCommand command = new VelocityCommand();
        Pair<Double, Boolean> vx = result.getDouble(); if (!vx.getSecond()) { System.out.println("Failed to get vx"); return new VelocityCommand(); }
        Pair<Double, Boolean> vy = result.getDouble(); if (!vy.getSecond()) { System.out.println("Failed to get vy"); return new VelocityCommand(); }
        Pair<Double, Boolean> vz = result.getDouble(); if (!vz.getSecond()) { System.out.println("Failed to get vz"); return new VelocityCommand(); }
        command.vx = vx.getFirst();
        command.vy = vy.getFirst();
        command.vt = vz.getFirst();
        return command;
    }

    private Pose2d parsePose2d(PacketResult result) {
        Pair<Double, Boolean> x = result.getDouble(); if (!x.getSecond()) { System.out.println("Failed to get pose x"); return new Pose2d(); }
        Pair<Double, Boolean> y = result.getDouble(); if (!y.getSecond()) { System.out.println("Failed to get pose y"); return new Pose2d(); }
        Pair<Double, Boolean> theta = result.getDouble(); if (!theta.getSecond()) { System.out.println("Failed to get pose theta"); return new Pose2d(); }
        return new Pose2d(x.getFirst(), y.getFirst(), new Rotation2d(theta.getFirst()));
    }

    private void parseJoint(PacketResult result) {
        Pair<Integer, Boolean> index_pair = result.getInt(); if (!index_pair.getSecond()) { System.out.println("Failed to get joint index"); return; }
        Pair<Double, Boolean> position_pair = result.getDouble(); if (!position_pair.getSecond()) { System.out.println("Failed to get joint position"); return; }
        int index = index_pair.getFirst();
        double position = position_pair.getFirst();
        if (index > 255) {
            System.out.println("Maximum of 255 joints supported! Ignoring joint index " + index);
            return;
        }
        while (index >= jointCommandValues.size()) {
            jointCommandValues.add(0.0);
            jointCommandTimers.add(new MessageTimer(DEFAULT_MESSAGE_TIMEOUT));
        }
        jointCommandValues.set(index, position);
        jointCommandTimers.get(index).reset();
    }

    public void update()
    {
        data_stream.update();

        writeOdom();

        sendMatchStatus(
            DriverStation.isAutonomous(),
            DriverStation.getMatchTime(),
            DriverStation.getAlliance()
        );
    }

    private void writeOdom() {
        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisVelocity();

        data_stream.writePacket("odom",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond
        );
    }

    public void sendMatchStatus(boolean is_autonomous, double match_timer, DriverStation.Alliance team_color) {
        data_stream.writePacket("match",
            is_autonomous, match_timer, getTeamName(team_color)
        );
    }

    protected void writeImu(AHRS imu) {
        data_stream.writePacket("imu",
            (double)(imu.getRawAccelX()),
            (double)(imu.getRawAccelY()),
            (double)(imu.getRawAccelZ()),
            (double)(imu.getRawGyroX()),
            (double)(imu.getRawGyroY()),
            (double)(imu.getRawGyroZ()),
            (double)(imu.getRoll()),
            (double)(imu.getPitch()),
            (double)(imu.getYaw())
        );
    }
    protected void writeImu2d(AHRS imu) {
        data_stream.writePacket("imu",
            imu.getRate(),
            imu.getYaw()
        );
    }

    protected void writePose2d(String category, Pose2d pose) {
        data_stream.writePacket(category,
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        );
    }

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        String waypointName = WaypointMap.parseWaypointName(waypoint.waypoint_name);
        
        data_stream.writePacket("waypoint", 
            numSentGoals, waypointName,
            waypoint.pose.getX(),
            waypoint.pose.getY(),
            waypoint.pose.getRotation().getRadians(),
            waypoint.intermediate_tolerance,
            waypoint.timeout,
            createWaypointOptionBitmask(waypoint)
        );
        numSentGoals++;
    }

    private int createWaypointOptionBitmask(Waypoint waypoint) {
        int bitmask = 0;
        bitmask |= boolToBitmask(0, waypoint.is_continuous);
        bitmask |= boolToBitmask(1, waypoint.ignore_orientation);
        bitmask |= boolToBitmask(2, waypoint.ignore_obstacles);
        bitmask |= boolToBitmask(3, waypoint.ignore_walls);
        return bitmask;
    }

    private int boolToBitmask(int shift, boolean value) {
        return (value ? 1 : 0) << shift;
    }

    public void executeGoal() {
        data_stream.writePacket("execute");
        numSentGoals = 0;
    }

    public void cancelGoal() {
        data_stream.writePacket("cancel");
    }
    
    public void resetPlan() {
        data_stream.writePacket("reset");
        numSentGoals = 0;
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        writePose2d("pose_est", poseEstimation);
    }

    public void setJointPosition(int index, double position) {
        data_stream.writePacket("joint", index, position);
    }
}
