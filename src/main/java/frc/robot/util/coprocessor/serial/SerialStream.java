// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor.serial;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.coprocessor.tunnel.DataStreamInterface;
import frc.robot.util.coprocessor.tunnel.Handshake;
import frc.robot.util.coprocessor.tunnel.PacketResult;
import frc.robot.util.coprocessor.tunnel.PacketType;
import frc.robot.util.coprocessor.tunnel.TunnelInterface;
import frc.robot.util.coprocessor.tunnel.TunnelProtocol;


public class SerialStream implements DataStreamInterface {
    SerialPort serial;
    private TunnelProtocol protocol;
    private TunnelInterface tunnel_interface;

    private byte[] buffer = new byte[TunnelProtocol.MAX_PACKET_LEN << 4];
    private int unparsed_index = 0;
    private ArrayList<Handshake> pending_handshakes = new ArrayList<>();
    
    public SerialStream(TunnelInterface tunnel_interface, int baud) {
        serial = new SerialPort(baud, SerialPort.Port.kMXP);
        serial.disableTermination();

        protocol = new TunnelProtocol();
        this.tunnel_interface = tunnel_interface;
    }

    public SerialStream(TunnelInterface tunnel_interface) {
        this(tunnel_interface, 115200);
    }


    public void writePacket(PacketType type, String category, Object... objects) {
        byte[] data = protocol.makePacket(type, category, objects);
        writeBuffer(data);
    }
    
    public void writePacket(String category, Object... objects) {
        writePacket(PacketType.NORMAL, category, objects);
    }

    public void writeHandshakePacket(String category, long write_interval, long timeout, Object... objects) {
        Handshake handshake = new Handshake(category, protocol.getWritePacketNum(), write_interval, timeout);
        byte[] data = protocol.makePacket(PacketType.HANDSHAKE, category, objects);
        handshake.setPacket(data);
        writeBuffer(data);
        pending_handshakes.add(handshake);
    }

    public void writeBuffer(byte[] buffer) {
        serial.write(buffer, buffer.length);
    }

    public void writeConfirmingPacket(String category, int packet_num, int error_code) {
        writePacket(PacketType.CONFIRMING, category, packet_num, error_code);
    }

    public long getTime() {
        return RobotController.getFPGATime();
    }

    void check_handshakes()
    {
        for (Handshake handshake : pending_handshakes) {
            if (handshake.shouldWriteAgain(getTime())) {
                writeBuffer(handshake.getPacket());
            }
            if (handshake.didFail(getTime())) {
                System.out.println(String.format("Handshake packet failed. category: %s #%d.", handshake.getCategory(), handshake.getPacketNum()));
                pending_handshakes.remove(handshake);
            }
        }
    }

    public void update() {
        check_handshakes();
        int length = serial.getBytesReceived();
        if (length == 0) {
            return;
        }
        byte[] data = serial.read(length);

        // Move stop index the length of the new data
        int buffer_stop = unparsed_index + data.length;
        if (buffer_stop > buffer.length) {
            System.out.println("Buffer overrun! Resetting");
            buffer_stop = 0;
            unparsed_index = 0;
            serial.flush();
            return;
        }

        // Append newly arrived bytes to buffer
        for (int index = 0; index < data.length; index++) {
            buffer[unparsed_index + index] = data[index];
        }

        // Attempt to parse the entire buffer.
        // last_parsed_index marks the last packet parsed. If no packets were parsed, this is 0
        int last_parsed_index = protocol.parseBuffer(buffer, 0, buffer_stop);
        if (last_parsed_index > 0)
        {
            // If at least one packet was parsed, remove this data from the buffer and shift all bytes over
            for (int index = last_parsed_index, shifted_index = 0; index < buffer_stop; index++, shifted_index++) {
                buffer[shifted_index] = buffer[index];
            }
        }
        // shift the buffer stop index by the amount we parsed
        unparsed_index = buffer_stop - last_parsed_index;
        if (unparsed_index >= TunnelProtocol.MAX_PACKET_LEN || unparsed_index < 0) {
            // if we somehow parsed more than the allowed amount, reset
            unparsed_index = 0;
        }

        // Results are stored in protocol object. Iterate through the results and run the callback
        PacketResult result;
        do {
            result = protocol.popResult();
            if (result.getErrorCode() == TunnelProtocol.NULL_ERROR) {
                continue;
            }
            if (protocol.isCodeError(result.getErrorCode())) {
                System.out.println(String.format("Encountered error code %d.",
                    result.getErrorCode()
                ));
                continue;
            }
            if (result.getPacketType() == PacketType.HANDSHAKE) {
                writeConfirmingPacket(result.getCategory(), result.getPacketNum(), result.getErrorCode());
            }
            else if (result.getPacketType() == PacketType.CONFIRMING) {
                Pair<Integer, Boolean> packet_num_pair = result.getInt(); if (!packet_num_pair.getSecond()) { System.out.println("Failed to get confirm packet num"); continue; }
                Pair<Integer, Boolean> error_code_pair = result.getInt(); if (!error_code_pair.getSecond()) { System.out.println("Failed to get confirm packet error code"); continue; }
    
                if (protocol.isCodeError(error_code_pair.getFirst())) {
                    System.out.println(String.format("Handshake confirm has an error code. category %s #%d: %d", result.getCategory(), result.getPacketNum(), result.getErrorCode()));
                    continue;
                }
                boolean handshake_found = false;
                for (int index = 0; index < pending_handshakes.size(); index++) {
                    Handshake handshake = pending_handshakes.get(index);
                    if (handshake_found) {
                        System.out.println(String.format("Encountered duplicate handshake. category: %s #%d.", handshake.getCategory(), handshake.getPacketNum()));
                    }
                    if (handshake.isEqual(result.getCategory(), packet_num_pair.getFirst())) {
                        pending_handshakes.remove(index);
                        index--;
                        handshake_found = true;
                        tunnel_interface.handshakeCallback(this, handshake);
                    }
                }
                continue;
            }
            tunnel_interface.packetCallback(this, result);
        }
        while (result.getErrorCode() != -1);
    }
}
