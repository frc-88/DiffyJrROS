package frc.robot.util.coprocessor.networktables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedStringArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.Helpers;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.coprocessor.roswaypoints.GoalStatus;
import frc.robot.util.coprocessor.roswaypoints.Waypoint;
import frc.robot.util.coprocessor.CoprocessorBase;

public class CoprocessorTable extends CoprocessorBase {
    private NetworkTableInstance instance;
    private String address;
    private int port;
    protected NetworkTable rootTable;
    private double updateInterval = 0.01;
    private DoubleSubscriber pingSub;
    private DoublePublisher pingReturnPub;

    private DoubleArrayPublisher odomPub;
    private DoubleArraySubscriber cmdVelSub;
    private DoubleArraySubscriber globalPoseSub;
    private DoubleArrayPublisher poseEstPub;
    private DoubleArrayPublisher imuPub;

    private NetworkTable matchTable;
    private DoublePublisher matchTimerPub;
    private BooleanPublisher isAutonomousPub;
    private StringPublisher teamColorPub;

    private NetworkTable jointsTable;
    private DoubleArrayPublisher jointsPub;
    private DoubleArraySubscriber jointCommandsSub;

    private NetworkTable waypointsTable;
    private StringArraySubscriber waypointNamesSub;
    private DoubleArraySubscriber waypointsXSub;
    private DoubleArraySubscriber waypointsYSub;
    private DoubleArraySubscriber waypointsTSub;

    private NetworkTable laserScanTable;
    private DoubleArraySubscriber laserScanXSub;
    private DoubleArraySubscriber laserScanYSub;

    private NetworkTable zonesTable;
    private NetworkTable zoneInfoTable;
    private BooleanSubscriber zoneInfoIsValidSub;
    private StringArraySubscriber zoneInfoNamesSub;
    private DoubleArraySubscriber zoneNearestXSub;
    private DoubleArraySubscriber zoneNearestYSub;
    private DoubleArraySubscriber zoneDistanceSub;
    private BooleanArraySubscriber zoneIsInsideSub;
    private BooleanArraySubscriber zoneIsNogoSub;
    private StringArrayPublisher zoneNoGoNamesPub;

    public CoprocessorTable(ChassisInterface chassis, String address, int port, double updateInterval) {
        super(chassis);
        this.address = address;
        this.port = port;

        instance = NetworkTableInstance.create();
        instance.startClient4("coprocessor");
        instance.setServer(new String[] {address}, port);
        this.updateInterval = updateInterval;

        rootTable = instance.getTable("ROS");
        
        pingSub = rootTable.getDoubleTopic("ping").subscribe(0.0, PubSubOption.periodic(this.updateInterval));
        pingReturnPub = rootTable.getDoubleTopic("ping_return").publish(PubSubOption.periodic(this.updateInterval));

        odomPub = rootTable.getDoubleArrayTopic("odom").publish(PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
        cmdVelSub = rootTable.getDoubleArrayTopic("cmd_vel").subscribe(new double []{0.0, 0.0, 0.0}, PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
        globalPoseSub = rootTable.getDoubleArrayTopic("global").subscribe(new double []{0.0, 0.0, 0.0}, PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));
        poseEstPub = rootTable.getDoubleArrayTopic("pose_est").publish(PubSubOption.sendAll(true));
        imuPub = rootTable.getDoubleArrayTopic("imu").publish(PubSubOption.sendAll(true), PubSubOption.periodic(this.updateInterval));

        matchTable = rootTable.getSubTable("match");
        matchTimerPub = matchTable.getDoubleTopic("time").publish();
        isAutonomousPub = matchTable.getBooleanTopic("is_auto").publish();
        teamColorPub = matchTable.getStringTopic("team_color").publish();
        
        jointsTable = rootTable.getSubTable("joints");
        jointsPub = jointsTable.getDoubleArrayTopic("states").publish();
        jointCommandsSub = jointsTable.getDoubleArrayTopic("commands").subscribe(new double []{});

        waypointsTable = rootTable.getSubTable("waypoints");
        waypointNamesSub = waypointsTable.getStringArrayTopic("name").subscribe(new String []{});
        waypointsXSub = waypointsTable.getDoubleArrayTopic("x").subscribe(new double []{});
        waypointsYSub = waypointsTable.getDoubleArrayTopic("y").subscribe(new double []{});
        waypointsTSub = waypointsTable.getDoubleArrayTopic("t").subscribe(new double []{});

        laserScanTable = rootTable.getSubTable("laser");
        laserScanXSub = laserScanTable.getDoubleArrayTopic("xs").subscribe(new double []{});
        laserScanYSub = laserScanTable.getDoubleArrayTopic("ys").subscribe(new double []{});

        zonesTable = rootTable.getSubTable("zones");
        zoneInfoTable = zonesTable.getSubTable("info");
        zoneInfoIsValidSub = zoneInfoTable.getBooleanTopic("is_valid").subscribe(false);
        zoneInfoNamesSub = zoneInfoTable.getStringArrayTopic("names").subscribe(new String []{});
        zoneNearestXSub = zoneInfoTable.getDoubleArrayTopic("nearest_x").subscribe(new double []{});
        zoneNearestYSub = zoneInfoTable.getDoubleArrayTopic("nearest_y").subscribe(new double []{});
        zoneDistanceSub = zoneInfoTable.getDoubleArrayTopic("distance").subscribe(new double []{});
        zoneIsInsideSub = zoneInfoTable.getBooleanArrayTopic("is_inside").subscribe(new boolean []{});
        zoneIsNogoSub = zoneInfoTable.getBooleanArrayTopic("is_nogo").subscribe(new boolean []{});
        zoneNoGoNamesPub = zonesTable.getStringArrayTopic("set_nogo").publish();
    }

    private void updatePing() {
        TimestampedDouble ping = pingSub.getAtomic();
        if (ping.timestamp == 0.0) {
            return;
        }
        pingReturnPub.set(ping.value);
    }

    private void sendOdometry(Pose2d pose, ChassisSpeeds velocity) {
        odomPub.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond
        });
    }

    private void updateCmdVel() {
        TimestampedDoubleArray cmd = cmdVelSub.getAtomic();
        if (cmd.timestamp == 0.0) {
            return;
        }
        if (cmd.value.length != 3) {
            System.out.println("Warning: Received command is not of length 3. Ignoring.");
            return;
        }
        command.vx = cmd.value[0];
        command.vy = cmd.value[1];
        command.vt = cmd.value[2];
        commandTimer.reset();
    }

    private void updateGlobalPose() {
        TimestampedDoubleArray pose = globalPoseSub.getAtomic();
        if (pose.timestamp == 0.0) {
            return;
        }
        if (pose.value.length != 3) {
            System.out.println("Warning: Received global pose is not of length 3. Ignoring.");
            return;
        }
        double x = pose.value[0];
        double y = pose.value[1];
        double theta = pose.value[2];
        globalPose = new Pose2d(x, y, new Rotation2d(theta));
        globalPoseTimer.reset();
        commandTimer.reset();
    }

    public void sendImu(double roll, double pitch, double yaw, double angular_z, double accel_x, double accel_y) {
        imuPub.set(new double[] {
            roll,
            pitch,
            yaw,
            angular_z,
            accel_x,
            accel_y
        });
    }

    protected void sendMatchStatus(boolean is_autonomous, double match_timer, DriverStation.Alliance team_color) {
        isAutonomousPub.set(is_autonomous);
        teamColorPub.set(Helpers.getTeamColorName(team_color));
        matchTimerPub.set(match_timer);
    }

    private void sendJointStates() {
        jointsPub.set(jointStates.stream().mapToDouble(state -> state).toArray());
    }

    private void updateJointCommands() {
        TimestampedDoubleArray cmds = jointCommandsSub.getAtomic();
        if (cmds.timestamp == 0.0) {
            return;
        }
        if (cmds.value.length != jointStates.size()) {
            System.out.println("Warning: Received joint command that doesn't match states length. Ignoring.");
            return;
        }
        for (int index = 0; index < cmds.value.length; index++) {
            jointCommands.set(index, cmds.value[index]);
        }
        jointCommandTimer.reset();
    }

    private void updateWaypoints() {
        TimestampedStringArray names = waypointNamesSub.getAtomic();
        if (names.timestamp == 0.0) {
            return;
        }
        
        double[] xs = waypointsXSub.get();
        double[] ys = waypointsYSub.get();
        double[] ts = waypointsTSub.get();
        if (names.value.length != xs.length ||
                names.value.length != ys.length ||
                names.value.length != ts.length) {
            System.out.println("Warning: waypoint entries have mismatched lengths. Ignoring.");
            return;
        }

        for (int index = 0; index < names.value.length; index++) {
            double wayx = xs[index];
            double wayy = ys[index];
            double wayt = ts[index];
            putWaypoint(names.value[index], new Pose2d(wayx, wayy, new Rotation2d(wayt)));
        }
    }

    private void updateZones() {
        boolean is_valid = zoneInfoIsValidSub.get();
        if (!is_valid) {
            return;
        }
        TimestampedStringArray names = zoneInfoNamesSub.getAtomic();
        double[] nearest_xs = zoneNearestXSub.get();
        double[] nearest_ys = zoneNearestYSub.get();
        double[] distances = zoneDistanceSub.get();
        boolean[] is_insides = zoneIsInsideSub.get();
        boolean[] is_nogos = zoneIsNogoSub.get();
        if (names.value.length != nearest_xs.length ||
                names.value.length != nearest_ys.length ||
                names.value.length != distances.length ||
                names.value.length != is_insides.length ||
                names.value.length != is_nogos.length) {
            System.out.println("Warning: zone entries have mismatched lengths. Ignoring.");
            return;
        }
        for (int index = 0; index < names.value.length; index++) {
            String name = names.value[index];
            double nearest_x = nearest_xs[index];
            double nearest_y = nearest_ys[index];
            double distance = distances[index];
            boolean is_inside = is_insides[index];
            boolean is_nogo = is_nogos[index];
            zoneManager.setZone(
                name,
                nearest_x,
                nearest_y,
                distance,
                is_inside,
                is_nogo
            );
        }
    }

    private void updateNoGoZones() {
        Set<String> nogos = zoneManager.getNoGoNames();
        String values[] = new String[nogos.size()];
        int index = 0;
        for (String nogo : nogos) {
            values[index++] = nogo;
        }
        zoneNoGoNamesPub.set(values);
    }

    public void setNoGoZones(String[] names) {
        super.setNoGoZones(names);
        updateNoGoZones();
    }

    public void setNoGoZone(String name) {
        super.setNoGoZone(name);
        updateNoGoZones();
    }

    public void removeNoGoZone(String name) {
        super.removeNoGoZone(name);
        updateNoGoZones();
    }

    public void sendPoseEstimate(Pose2d poseEstimation) {
        poseEstPub.set(new double[] {
            poseEstimation.getX(),
            poseEstimation.getY(),
            poseEstimation.getRotation().getRadians()
        });
    }

    private void updateLaserScan()
    {
        TimestampedDoubleArray xs = laserScanXSub.getAtomic();
        if (xs.timestamp == 0.0) {
            return;
        }
        TimestampedDoubleArray ys = laserScanYSub.getAtomic();
        if (ys.timestamp == 0.0) {
            return;
        }
        if (xs.value.length != ys.value.length) {
            System.out.println("Warning: laser scan has mismatched lengths. Ignoring.");
            return;
        }

        laserObstacles.setPoints(xs.value, ys.value);
    }

    public void update() {
        if (!instance.isConnected()) {
            return;
        }

        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisSpeeds();

        updatePing();
        sendOdometry(pose, velocity);
        updateCmdVel();
        updateGlobalPose();
        sendJointStates();
        updateJointCommands();
        updateWaypoints();
        updateLaserScan();
        updateZones();

        sendMatchStatus(
            DriverStation.isAutonomous(),
            DriverStation.getMatchTime(),
            DriverStation.getAlliance()
        );
    }

    public boolean isConnected() {
        return instance.isConnected();
    }

    public double getUpdateInterval() {
        return updateInterval;
    }
}
