// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor.serial;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.coprocessor.tunnel.DataStreamInterface;
import frc.robot.util.coprocessor.tunnel.PacketResult;
import frc.robot.util.coprocessor.tunnel.TunnelInterface;
import frc.robot.util.coprocessor.tunnel.TunnelProtocol;


public class SerialStream implements DataStreamInterface {
    SerialPort serial;
    private TunnelProtocol protocol;
    private TunnelInterface tunnel_interface;

    private int buffer_size = 1024;
    private byte[] buffer = new byte[buffer_size];
    private int unparsed_index = 0;
    
    public SerialStream(TunnelInterface tunnel_interface, int baud) {
        serial = new SerialPort(baud, SerialPort.Port.kMXP);
        serial.disableTermination();

        protocol = new TunnelProtocol();
        this.tunnel_interface = tunnel_interface;
    }

    public SerialStream(TunnelInterface tunnel_interface) {
        this(tunnel_interface, 115200);
    }

    
    public void writePacket(String category, Object... objects) {
        byte[] data = protocol.makePacket(category, objects);
        serial.write(data, data.length);
    }

    public void update() {
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
        if (unparsed_index >= buffer_size || unparsed_index < 0) {
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
            tunnel_interface.packetCallback(this, result);
        }
        while (result.getErrorCode() != -1);
    }
}
