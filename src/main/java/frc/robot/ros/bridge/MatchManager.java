package frc.robot.ros.bridge;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ros.messages.tj2_interfaces.Match;
import frc.robot.ros.messages.tj2_interfaces.MatchPeriod;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;

public class MatchManager {
    private final BridgePublisher<Match> m_matchPub;
    private final BridgePublisher<MatchPeriod> m_matchPeriodPub;
    private MatchPeriod m_matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);

    public MatchManager(ROSNetworkTablesBridge bridge) {
        m_matchPub = new BridgePublisher<>(bridge, "match");
        m_matchPeriodPub = new BridgePublisher<>(bridge, "match_period");
    }

    public void sendDisableMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.DISABLED);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    public void sendAutonomousMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.AUTONOMOUS);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    public void sendTeleopMatchPeriod() {
        m_matchPeriod = new MatchPeriod(MatchPeriod.TELEOP);
        m_matchPeriodPub.send(m_matchPeriod);
    }

    public void sendMatch() {
        m_matchPub.send(new Match(
                DriverStation.getMatchTime(),
                DriverStation.getAlliance().name(),
                (byte) DriverStation.getLocation(),
                m_matchPeriod));
    }
}
