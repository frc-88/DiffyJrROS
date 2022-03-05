package frc.robot.util.tunnel;

import java.io.IOException;
import java.net.SocketException;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

public class TunnelWriteThread extends Thread {
    private TunnelClient client;
    private Queue<byte[]> write_queue;

    public TunnelWriteThread(TunnelClient client, ReentrantLock write_lock, Queue<byte[]> write_queue) {
        this.client = client;
        this.write_queue = write_queue;
    }

    public void run()
    {
        System.out.println("Starting write thread");
        while (true) {
            if (client.getShouldClose()) {
                break;
            }

            if (!dequeBuffer()) {
                break;
            }
        }
        client.setShouldClose(true);
        System.out.println("Stopping write thread");
    }

    private boolean writeBuffer(byte[] buffer) throws IOException
    {
        if (buffer.length == 0) {
            System.out.println("Buffer is empty. Skipping write.");
            return true;
        }
        if (Objects.isNull(client.output)) {
            System.out.println("Socket is closed! Skipping write.");
            return false;
        }
        try {
            client.output.write(buffer, 0, buffer.length);
            client.output.flush();
        }
        catch (SocketException e) {
            System.out.println("Failed while writing buffer: " + TunnelUtil.packetToString(buffer) + ". " + e.getMessage());
            return false;
        }
        return true;
    }

    public boolean dequeBuffer()
    {
        byte[] data = null;
        try {
            while (!write_queue.isEmpty()) {
                data = write_queue.poll();
                if (Objects.isNull(data)) {
                    continue;
                }
                if (!writeBuffer(data)) {
                    return false;
                }
            }
        }
        catch (IOException e) {
            if (Objects.isNull(data)) {
                System.out.println("Failed while writing data: " + TunnelUtil.packetToString(data) + ". " + e.getMessage());
            }
            else {
                System.out.println("Failed while writing uninitialized data. " + e.getMessage());
            }
        }
        return true;
    }

    public void queueBuffer(byte[] buffer) {
        write_queue.add(buffer);
        // try {
        //     if (!writeBuffer(buffer)) {
        //         client.setShouldClose(true);
        //     }
        // }
        // catch (IOException e) {
        //     System.out.println("Failed while writing uninitialized data. " + e.getMessage());
        // }
    }
}
