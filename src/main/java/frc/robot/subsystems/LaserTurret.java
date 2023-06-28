package frc.robot.subsystems;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferenceconstants.DoublePreferenceConstant;

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

    private final long UPDATE_INTERVAL = 40_000;
    private long last_update = 0;

    private ServoCommand servo1_command = new ServoCommand(1,
            new DoublePreferenceConstant("LaserTurret/default_tilt", 0).getValue());
    private ServoCommand servo2_command = new ServoCommand(2,
            new DoublePreferenceConstant("LaserTurret/default_pan", 0).getValue());
    private LaserCommand laser_command = new LaserCommand(false);

    public LaserTurret(SerialPort.Port port) {
        device = new SerialPort(115200, port);
    }

    public void setPanPosition(double new_angle) {
        SmartDashboard.putNumber("LaserTurret/command/pan", new_angle);
        servo2_command = new ServoCommand(2, new_angle);
    }

    public void setTiltPosition(double new_angle) {
        SmartDashboard.putNumber("LaserTurret/command/tilt", new_angle);
        servo1_command = new ServoCommand(1, new_angle);
    }

    public void setLaser(boolean new_state) {
        SmartDashboard.putBoolean("LaserTurret/command/laser", new_state);
        laser_command = new LaserCommand(new_state);
    }

    public double getPanPosition() {
        return state.servo2;
    }

    public double getTiltPosition() {
        return state.servo1;
    }

    public boolean isLaserOn() {
        return state.laser == 1 ? true : false;
    }

    private void writeString(String command) {
        System.out.println("Sending command: " + command);
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

    private void writeCommands() {
        writeString(ginst.toJson(servo1_command) + "\n"
                + ginst.toJson(servo2_command) + "\n"
                + ginst.toJson(laser_command));
    }

    @Override
    public void periodic() {
        int received = device.getBytesReceived();
        if (received > 0) {
            System.out.println("Received " + received + " bytes");
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

        long now = RobotController.getFPGATime();
        if (now - last_update > UPDATE_INTERVAL) {
            writeCommands();
            last_update = now;
        }
    }
}
