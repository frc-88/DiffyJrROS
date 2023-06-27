package frc.robot.subsystems;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserTurret extends SubsystemBase {

    public class State {
        public double servo1;
        public double servo2;
        public int laser;
    }

    public class ServoCommand {
        public int channel;
        public double angle;
        public String command = "servo";

        public ServoCommand(int channel, double angle) {
            this.channel = channel;
            this.angle = angle;
        }
    }

    public class LaserCommand {
        public boolean state;
        public String command = "laser";

        public LaserCommand(boolean state) {
            this.state = state;
        }
    }

    private final static Gson ginst = new Gson();
    private final SerialPort device;
    private State state = new State();
    private String buffer = "";

    public LaserTurret(SerialPort.Port port) {
        device = new SerialPort(9600, port);
        SmartDashboard.putBoolean("LaserTurret/command/laser", false);
        SmartDashboard.putNumber("LaserTurret/command/pan", 0);
        SmartDashboard.putNumber("LaserTurret/command/tilt", 0);
    }

    public void setPanPosition(double new_angle) {
        writeString(ginst.toJson(new ServoCommand(1, new_angle)));
    }

    public void setTiltPosition(double new_angle) {
        writeString(ginst.toJson(new ServoCommand(2, new_angle)));
    }

    public void setLaser(boolean new_state) {
        writeString(ginst.toJson(new LaserCommand(new_state)));
    }

    public double getPanPosition() {
        return state.servo1;
    }

    public double getTiltPosition() {
        return state.servo2;
    }

    public boolean isLaserOn() {
        return state.laser == 1 ? true : false;
    }

    private void writeString(String command) {
        device.writeString(command + "\n");
    }

    private void parsePacket(String packet) {
        State new_state = null;
        try {
            new_state = ginst.fromJson(packet, new TypeToken<State>() {
            }.getType());
        } catch (Exception e) {
            System.out.println("Failed to parse packet: " + packet);
        }
        if (new_state != null) {
            this.state = new_state;
            SmartDashboard.putBoolean("LaserTurret/state/laser", isLaserOn());
            SmartDashboard.putNumber("LaserTurret/state/pan", getPanPosition());
            SmartDashboard.putNumber("LaserTurret/state/tilt", getTiltPosition());
        }
    }

    @Override
    public void periodic() {
        boolean laser_state = SmartDashboard.getBoolean("LaserTurret/command/laser", false);
        if (laser_state != isLaserOn()) {
            setLaser(laser_state);
        }
        double pan_command = SmartDashboard.getNumber("LaserTurret/command/pan", 0);
        if (pan_command != getPanPosition()) {
            setPanPosition(pan_command);
        }
        double tilt_command = SmartDashboard.getNumber("LaserTurret/command/tilt", 0);
        if (tilt_command != getTiltPosition()) {
            setTiltPosition(tilt_command);
        }

        int received = device.getBytesReceived();
        if (received > 0) {
            String read_buffer = device.readString(received);
            buffer += read_buffer;
            String packets;
            if (buffer.charAt(buffer.length() - 1) == '\n') {
                packets = buffer;
                buffer = "";
            } else {
                int last_term_index = buffer.lastIndexOf('\n');
                if (last_term_index == -1) {
                    packets = "";
                } else {
                    packets = buffer.substring(0, last_term_index);
                    buffer = buffer.substring(last_term_index + 1);
                }
            }
            for (String packet : packets.split("\n")) {
                parsePacket(packet);
            }
        }
    }
}
