package frc.robot.calibration_pointer;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferenceconstants.DoublePreferenceConstant;
import frc.robot.ros.bridge.JointManager;

public class CalibrationPointer extends SubsystemBase {

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

    private boolean enableROSJointStates = true;
    private final JointManager jointManager;
    private final String pointer_pan_joint = "pointer_pan_joint";
    private final String pointer_tilt_joint = "pointer_tilt_joint";

    private final long UPDATE_INTERVAL = 40_000;
    private long last_update = 0;

    private ServoCommand servo1_command = new ServoCommand(1,
            new DoublePreferenceConstant("CalibrationPointer/default_tilt", 0).getValue());
    private ServoCommand servo2_command = new ServoCommand(2,
            new DoublePreferenceConstant("CalibrationPointer/default_pan", 0).getValue());
    private LaserCommand laser_command = new LaserCommand(false);

    private double pan_servo_value_1 = new DoublePreferenceConstant("CalibrationPointer/pan_servo_value_1", 0)
            .getValue();
    private double pan_servo_value_2 = new DoublePreferenceConstant("CalibrationPointer/pan_servo_value_2", 180)
            .getValue();
    private double pan_servo_angle_1 = new DoublePreferenceConstant("CalibrationPointer/pan_servo_angle_1", 0)
            .getValue();
    private double pan_servo_angle_2 = new DoublePreferenceConstant("CalibrationPointer/pan_servo_angle_2", 180)
            .getValue();
    private double tilt_servo_value_1 = new DoublePreferenceConstant("CalibrationPointer/tilt_servo_value_1", 0)
            .getValue();
    private double tilt_servo_value_2 = new DoublePreferenceConstant("CalibrationPointer/tilt_servo_value_2", 180)
            .getValue();
    private double tilt_servo_angle_1 = new DoublePreferenceConstant("CalibrationPointer/tilt_servo_angle_1", 0)
            .getValue();
    private double tilt_servo_angle_2 = new DoublePreferenceConstant("CalibrationPointer/tilt_servo_angle_2", 180)
            .getValue();

    public CalibrationPointer(SerialPort.Port port, JointManager jointPublisher) {
        this.device = new SerialPort(115200, port);
        this.jointManager = jointPublisher;
    }

    public void setPanAngle(double new_angle) {
        double command = mapPanServoAngleToValue(new_angle);
        SmartDashboard.putNumber("CalibrationPointer/command/pan", command);
        servo2_command = new ServoCommand(2, command);
    }

    public void setTiltAngle(double new_angle) {
        double command = mapTiltServoAngleToValue(new_angle);
        SmartDashboard.putNumber("CalibrationPointer/command/tilt", command);
        servo1_command = new ServoCommand(1, command);
    }

    public void setLaser(boolean new_state) {
        SmartDashboard.putBoolean("CalibrationPointer/command/laser", new_state);
        laser_command = new LaserCommand(new_state);
    }

    private double map(double value, double in_min, double in_max, double out_min, double out_max) {
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private double mapPanServoAngleToValue(double servo_value) {
        return map(servo_value, pan_servo_value_1, pan_servo_value_2, pan_servo_angle_1, pan_servo_angle_2);
    }

    private double mapPanServoValueToAngle(double servo_angle) {
        return map(servo_angle, pan_servo_angle_1, pan_servo_angle_2, pan_servo_value_1, pan_servo_value_2);
    }

    private double mapTiltServoAngleToValue(double servo_value) {
        return map(servo_value, tilt_servo_value_1, tilt_servo_value_2, tilt_servo_angle_1, tilt_servo_angle_2);
    }

    private double mapTiltServoValueToAngle(double servo_angle) {
        return map(servo_angle, tilt_servo_angle_1, tilt_servo_angle_2, tilt_servo_value_1, tilt_servo_value_2);
    }

    public double getPanValue() {
        return state.servo2;
    }

    public double getPanAngle() {
        return mapPanServoValueToAngle(getPanValue());
    }

    public double getTiltValue() {
        return state.servo1;
    }

    public double getTiltAngle() {
        return mapTiltServoValueToAngle(getTiltValue());
    }

    public void setEnableROSJoints(boolean enabled) {
        enableROSJointStates = enabled;
    }

    public boolean isLaserOn() {
        return state.laser == 1 ? true : false;
    }

    private void writeString(String command) {
        // System.out.println("Sending command: " + command);
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
            SmartDashboard.putBoolean("CalibrationPointer/state/laser", isLaserOn());
            SmartDashboard.putNumber("CalibrationPointer/state/pan", getPanValue());
            SmartDashboard.putNumber("CalibrationPointer/state/tilt", getTiltValue());
        }
    }

    private void writeCommands() {
        writeString(ginst.toJson(servo1_command) + "\n"
                + ginst.toJson(servo2_command) + "\n"
                + ginst.toJson(laser_command));
    }

    private void updateJointStates() {
        if (enableROSJointStates) {
            jointManager.sendJointPosition(pointer_pan_joint, getPanAngle());
            jointManager.sendJointPosition(pointer_tilt_joint, getTiltAngle());
            setPanAngle(jointManager.getJointCommand(pointer_pan_joint));
            setTiltAngle(jointManager.getJointCommand(pointer_tilt_joint));
        }
    }

    @Override
    public void periodic() {
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

        long now = RobotController.getFPGATime();
        if (now - last_update > UPDATE_INTERVAL) {
            writeCommands();
            updateJointStates();
            last_update = now;
        }
    }
}
