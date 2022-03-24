package frc.robot.util.roswaypoints;

import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.util.coprocessortable.CoprocessorTable;

public class WaypointMap {
    private final NetworkTable table;
    private NewWaypointInterface m_callback;
    private final CoprocessorTable m_coprocessor;
    public WaypointMap(CoprocessorTable coprocessor, NewWaypointInterface callback) {
        m_coprocessor = coprocessor;
        table = m_coprocessor.getWaypointsTable();
        table.addSubTableListener((parent, name, table) -> {newWaypointCallback(name);}, true);
        m_callback = callback;
    }
    public WaypointMap(CoprocessorTable coprocessor) {
        m_coprocessor = coprocessor;
        table = m_coprocessor.getWaypointsTable();
    }

    private void newWaypointCallback(String name) {
        if (Objects.nonNull(m_callback)) {
            m_callback.newWaypointCallback(this, name);
        }
    }
    public Set<String> getWaypointNames() {
        return table.getSubTables();
    }
    public boolean doesWaypointExist(String waypointName) {
        return table.containsSubTable(waypointName);
    }
    public Pose2d getWaypoint(String waypointName) {
        if (doesWaypointExist(waypointName)) {
            double x = table.getSubTable(waypointName).getEntry("x").getDouble(Double.NaN);
            double y = table.getSubTable(waypointName).getEntry("y").getDouble(Double.NaN);
            double theta = table.getSubTable(waypointName).getEntry("theta").getDouble(Double.NaN);
            return new Pose2d(x, y, new Rotation2d(theta));
        }
        else {
            return new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
        }
    }
    public boolean isPoseValid(Pose2d pose) {
        return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians());
    }
}
