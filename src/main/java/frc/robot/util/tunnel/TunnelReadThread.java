package frc.robot.util.tunnel;

import java.io.IOException;
import java.util.Arrays;
import java.util.Objects;

public class TunnelReadThread extends Thread {
    private TunnelClient client;

    private int buffer_size = 1024;
    private byte[] buffer = new byte[buffer_size];
    private int unparsed_index = 0;
    
    public TunnelReadThread(TunnelClient client) {
        this.client = client;
    }

    public void run()
    {
        if (Objects.isNull(client.input)) {
            return;
        }

        System.out.println("Starting read thread");
        while (true) {
            if (client.getShouldCloseThreads()) {
                break;
            }
            try {
                int num_chars_read = client.input.read(buffer, unparsed_index, buffer_size - unparsed_index);
                if (num_chars_read == 0) {
                    continue;
                }
                if (num_chars_read == -1) {
                    System.out.println("No more data. Closing client");
                    client.socket.close();
                    break;
                }
                
                int buffer_stop = unparsed_index + num_chars_read;
                int last_parsed_index = client.protocol.parseBuffer(Arrays.copyOfRange(buffer, 0, buffer_stop));
                if (last_parsed_index > 0)
                {
                    for (int index = last_parsed_index, shifted_index = 0; index < buffer_stop; index++, shifted_index++) {
                        buffer[shifted_index] = buffer[index];
                    }
                }
                unparsed_index = buffer_stop - last_parsed_index;
                if (unparsed_index >= buffer_size) {
                    unparsed_index = 0;
                }
                
                PacketResult result;
                do {
                    result = client.protocol.popResult();
                    if (result.getErrorCode() == TunnelProtocol.NULL_ERROR) {
                        continue;
                    }
                    if (client.protocol.isCodeError(result.getErrorCode())) {
                        System.out.println(String.format("Encountered error code %d.",
                            result.getErrorCode()
                        ));
                        continue;
                    }
                    client.tunnel_interface.packetCallback(client, result);
                }
                while (result.getErrorCode() != -1);

            } catch (IOException e) {
                e.printStackTrace();
                client.setIsOpen(false);
                break;
            }
        }
        System.out.println("Stopping read thread");
    }
}
