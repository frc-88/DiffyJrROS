// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor.serial;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.tunnel.CoprocessorBase;
import frc.robot.util.coprocessor.tunnel.DataStreamInterface;
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
            goalStatus = GoalStatus.getStatus(result.getInt());
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
            String waypoint_name = result.getString();
            Pose2d waypoint_pose = parsePose2d(result);
            waypoints.put(waypoint_name, waypoint_pose);
        }
    }

    private VelocityCommand parseVelocityCommand(PacketResult result) {
        VelocityCommand command = new VelocityCommand();
        command.vx = result.getDouble();
        command.vy = result.getDouble();
        command.vt = result.getDouble();
        return command;
    }

    private Pose2d parsePose2d(PacketResult result) {
        Pose2d pose = new Pose2d(
            result.getDouble(),
            result.getDouble(),
            new Rotation2d(result.getDouble())
        );
        return pose;
    }

    private void parseJoint(PacketResult result) {
        int index = result.getInt();
        double position = result.getDouble();
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
            imu.getRawAccelX(),
            imu.getRawAccelY(),
            imu.getRawAccelZ(),
            imu.getRawGyroX(),
            imu.getRawGyroY(),
            imu.getRawGyroZ(),
            imu.getRoll(),
            imu.getPitch(),
            imu.getYaw()
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
            waypoint.interruptableBy,
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
