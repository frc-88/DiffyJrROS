package frc.robot.util.coprocessor.networktables;

import java.util.Collection;
import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.Helpers;
import frc.robot.util.coprocessor.LaserScanObstacleTracker;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.ZoneInfo;
import frc.robot.util.coprocessor.ZoneManager;
import frc.robot.util.coprocessor.detections.Detection;
import frc.robot.util.coprocessor.CoprocessorBase;

public class CoprocessorTable extends CoprocessorBase {
    private NetworkTableInstance instance;
    protected NetworkTable rootTable;
    protected double updateInterval = 0.01;
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
    private NetworkTableEntry waypointNamesSub;
    private DoubleArraySubscriber waypointsXSub;
    private DoubleArraySubscriber waypointsYSub;
    private DoubleArraySubscriber waypointsTSub;

    private NetworkTable laserScanTable;
    private DoubleArraySubscriber laserScanXSub;
    private DoubleArraySubscriber laserScanYSub;

    private NetworkTable zonesTable;
    private NetworkTable zoneInfoTable;
    private BooleanSubscriber zoneInfoIsValidSub;
    private NetworkTableEntry zoneInfoNamesSub;
    private DoubleArraySubscriber zoneNearestXSub;
    private DoubleArraySubscriber zoneNearestYSub;
    private DoubleArraySubscriber zoneDistanceSub;
    private DoubleArraySubscriber zoneIsInsideSub;
    private DoubleArraySubscriber zoneIsNogoSub;
    private DoublePublisher zoneNoGoUpdatePub;
    private StringArrayPublisher zoneNoGoNamesPub;

    private NetworkTable detectionsTable;

    public CoprocessorTable(ChassisInterface chassis, String address, int port, double updateInterval) {
        super(chassis);

        instance = NetworkTableInstance.create();
        instance.startClient3("coprocessor");
        instance.setServer(address, port);
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
        waypointNamesSub = waypointsTable.getEntry("name");
        waypointsXSub = waypointsTable.getDoubleArrayTopic("x").subscribe(new double []{});
        waypointsYSub = waypointsTable.getDoubleArrayTopic("y").subscribe(new double []{});
        waypointsTSub = waypointsTable.getDoubleArrayTopic("t").subscribe(new double []{});

        laserScanTable = rootTable.getSubTable("laser");
        laserScanXSub = laserScanTable.getDoubleArrayTopic("xs").subscribe(new double []{});
        laserScanYSub = laserScanTable.getDoubleArrayTopic("ys").subscribe(new double []{});

        zonesTable = rootTable.getSubTable("zones");
        zoneInfoTable = zonesTable.getSubTable("info");
        zoneInfoIsValidSub = zoneInfoTable.getBooleanTopic("is_valid").subscribe(false);
        zoneInfoNamesSub = zoneInfoTable.getEntry("names");
        zoneNearestXSub = zoneInfoTable.getDoubleArrayTopic("nearest_x").subscribe(new double []{});
        zoneNearestYSub = zoneInfoTable.getDoubleArrayTopic("nearest_y").subscribe(new double []{});
        zoneDistanceSub = zoneInfoTable.getDoubleArrayTopic("distance").subscribe(new double []{});
        zoneIsInsideSub = zoneInfoTable.getDoubleArrayTopic("is_inside").subscribe(new double []{});
        zoneIsNogoSub = zoneInfoTable.getDoubleArrayTopic("is_nogo").subscribe(new double []{});
        zoneNoGoUpdatePub = zonesTable.getDoubleTopic("update").publish();
        zoneNoGoNamesPub = zonesTable.getStringArrayTopic("set_nogo").publish();

        detectionsTable = rootTable.getSubTable("detections");
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
            getTime(),
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond,
            velocity.omegaRadiansPerSecond
        });
    }

    public boolean isCommandActive() {
        updateCmdVel();
        return commandTimer.isActive();
    }

    private void updateCmdVel() {
        TimestampedDoubleArray cmd = cmdVelSub.getAtomic();
        if (cmd.timestamp == 0.0) {
            return;
        }
        if (cmd.value.length != 4) {
            System.out.println("Warning: Received command is not of length 4. Ignoring.");
            return;
        }
        // index 0 is timestamp
        command.vx = cmd.value[1];
        command.vy = cmd.value[2];
        command.vt = cmd.value[3];
        commandTimer.reset();
    }

    private void updateGlobalPose() {
        TimestampedDoubleArray pose = globalPoseSub.getAtomic();
        if (pose.timestamp == 0.0) {
            return;
        }
        if (pose.value.length != 4) {
            System.out.println("Warning: Received global pose is not of length 4. Ignoring.");
            return;
        }
        // index 0 is timestamp
        double x = pose.value[1];
        double y = pose.value[2];
        double theta = pose.value[3];
        globalPose = new Pose2d(x, y, new Rotation2d(theta));
        globalPoseTimer.reset();
        commandTimer.reset();
    }

    public boolean isGlobalPoseActive() {
        updateGlobalPose();
        return super.isGlobalPoseActive();
    }


    public void sendImu(double roll, double pitch, double yaw, double angular_z, double accel_x, double accel_y) {
        imuPub.set(new double[] {
            getTime(),
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
        if (jointStates.size() == 0) {
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
    public void setJointPosition(int index, double position) {
        super.setJointPosition(index, position);
        sendJointStates();
    }

    public boolean isJointCommandActive(int jointIndex) {
        updateJointCommands();
        return super.isJointCommandActive(jointIndex);
    }

    public Pose2d getWaypoint(String waypointName) {
        updateWaypoints();
        return super.getWaypoint(waypointName);
    }

    public Set<String> getWaypointNames() {
        updateWaypoints();
        return waypoints.keySet();
    }

    private void updateWaypoints() {
        TimestampedDoubleArray xs_value = waypointsXSub.getAtomic();
        if (xs_value.timestamp == 0.0) {
            return;
        }
        
        String[] names = waypointNamesSub.getStringArray(new String[] {});
        double[] xs = xs_value.value;
        double[] ys = waypointsYSub.get();
        double[] ts = waypointsTSub.get();
        if (names.length != xs.length ||
                names.length != ys.length ||
                names.length != ts.length) {
            System.out.println("Warning: waypoint entries have mismatched lengths. Ignoring.");
            return;
        }

        for (int index = 0; index < names.length; index++) {
            double wayx = xs[index];
            double wayy = ys[index];
            double wayt = ts[index];
            putWaypoint(names[index], new Pose2d(wayx, wayy, new Rotation2d(wayt)));
        }
    }

    public boolean doesDetectionNameExist(String name) {
        updateDetections();
        return super.doesDetectionNameExist(name);
    }
    public boolean doesDetectionExist(String name, int index) {
        updateDetections();
        return super.doesDetectionExist(name, index);
    }

    public Detection getDetection(String name, int index) {
        updateDetections();
        return super.getDetection(name, index);
    }

    public Collection<Detection> getAllDetectionsNamed(String name) {
        updateDetections();
        return super.getAllDetectionsNamed(name);
    }

    public Collection<Detection> getAllDetections() {
        updateDetections();
        return super.getAllDetections();
    }

    private void updateDetections() {
        for (String name : detectionsTable.getSubTables()) {
            NetworkTable detectionType = detectionsTable.getSubTable(name);
            for (String index : detectionType.getSubTables()) {
                int index_number;
                try {
                    index_number = Integer.parseInt(index);
                }
                catch (java.lang.NumberFormatException e) {
                    System.out.println(String.format("Failed to parse detection %s index %s", name, index));
                    index_number = 0;
                }
                Detection detection = new Detection(
                    name,
                    index_number,
                    detectionType.getEntry(index + "/position/x").getDouble(0.0),
                    detectionType.getEntry(index + "/position/y").getDouble(0.0),
                    detectionType.getEntry(index + "/position/z").getDouble(0.0),
                    detectionType.getEntry(index + "/orientation/w").getDouble(1.0),
                    detectionType.getEntry(index + "/orientation/x").getDouble(0.0),
                    detectionType.getEntry(index + "/orientation/y").getDouble(0.0),
                    detectionType.getEntry(index + "/orientation/z").getDouble(0.0)
                );
                detectionManager.setDetection(name, index_number, detection);
            }
        }
    }

    private void updateZones() {
        boolean is_valid = zoneInfoIsValidSub.get();
        if (!is_valid) {
            return;
        }
        if (Objects.isNull(zoneInfoNamesSub) || !zoneInfoNamesSub.isValid()) {
            System.out.println("Zone names sub is not valid!");
            return;
        }
        TimestampedDoubleArray nearest_xs_value = zoneNearestXSub.getAtomic();
        if (nearest_xs_value.timestamp == 0.0) {
            return;
        }
        String[] names = zoneInfoNamesSub.getStringArray(new String[] {});
        double[] nearest_xs = nearest_xs_value.value;
        double[] nearest_ys = zoneNearestYSub.get();
        double[] distances = zoneDistanceSub.get();
        double[] is_insides = zoneIsInsideSub.get();
        double[] is_nogos = zoneIsNogoSub.get();

        if (names.length != nearest_xs.length ||
                names.length != nearest_ys.length ||
                names.length != distances.length ||
                names.length != is_insides.length ||
                names.length != is_nogos.length) {
            System.out.println("Warning: zone entries have mismatched lengths. Ignoring.");
            return;
        }
        for (int index = 0; index < names.length; index++) {
            String name = names[index];
            double nearest_x = nearest_xs[index];
            double nearest_y = nearest_ys[index];
            double distance = distances[index];
            boolean is_inside = is_insides[index] == 1.0;
            boolean is_nogo = is_nogos[index] == 1.0;
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
        zoneNoGoUpdatePub.set(getTime());
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
            getTime(),
            poseEstimation.getX(),
            poseEstimation.getY(),
            poseEstimation.getRotation().getRadians()
        });
    }

    public LaserScanObstacleTracker getLaserScanObstacles() {
        updateLaserScan();
        return laserObstacles;
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

    public boolean areZonesValid() {
        updateZones();
        return zoneManager.isValid();
    }

    public void update() {
        if (!instance.isConnected()) {
            return;
        }

        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisSpeeds();

        updatePing();
        sendOdometry(pose, velocity);

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
