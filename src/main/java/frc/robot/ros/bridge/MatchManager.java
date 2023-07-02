package frc.robot.ros.bridge;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ros.messages.tj2_interfaces.Match;
import frc.robot.ros.messages.tj2_interfaces.MatchPeriod;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;

public class MatchManager {
    private final BridgePublisher<Match> matchPub;
    private final BridgePublisher<MatchPeriod> matchPeriodPub;
    private MatchPeriod matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);

    public MatchManager(ROSNetworkTablesBridge bridge) {
        matchPub = new BridgePublisher<>(bridge, "match");
        matchPeriodPub = new BridgePublisher<>(bridge, "match_period");
    }

    public void sendDisableMatchPeriod() {
        matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);
        matchPeriodPub.send(matchPeriod);
    }

    public void sendAutonomousMatchPeriod() {
        matchPeriod = new MatchPeriod(MatchPeriod.AUTONOMOUS);
        matchPeriodPub.send(matchPeriod);
    }

    public void sendTeleopMatchPeriod() {
        matchPeriod = new MatchPeriod(MatchPeriod.TELEOP);
        matchPeriodPub.send(matchPeriod);
    }

    public void sendMatch() {
        matchPub.send(new Match(
                DriverStation.getMatchTime(),
                DriverStation.getAlliance().name(),
                (byte) DriverStation.getLocation(),
                matchPeriod));
    }
}
