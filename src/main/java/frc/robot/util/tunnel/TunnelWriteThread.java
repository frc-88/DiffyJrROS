package frc.robot.util.tunnel;

import java.io.IOException;
import java.net.SocketException;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

public class TunnelWriteThread extends Thread {
    private TunnelClient client;

    private static ReentrantLock write_lock = new ReentrantLock();
    private static ReentrantLock queue_lock = new ReentrantLock();
    private static Queue<byte[]> write_queue = new LinkedList<byte[]>();

    
    public TunnelWriteThread(TunnelClient client) {
        this.client = client;
    }


    public void run()
    {
        System.out.println("Starting write thread");
        while (true) {
            if (client.getShouldCloseThreads()) {
                break;
            }
            dequeBuffer();
        }
        System.out.println("Stopping write thread");
    }

    private void writeBuffer(byte[] buffer) throws IOException
    {
        if (buffer.length == 0) {
            System.out.println("Buffer is empty. Skipping write.");
            return;
        }
        if (Objects.isNull(client.output) || !client.isOpen()) {
            System.out.println("Socket is closed! Skipping write.");
            return;
        }
        try {
            client.output.write(buffer, 0, buffer.length);
            client.output.flush();
        }
        catch (SocketException e) {
            e.printStackTrace();
            client.setIsOpen(false);
            System.out.println("Failed while writing buffer: " + TunnelUtil.packetToString(buffer));
        }
    }

    public void queueBuffer(byte[] buffer) {
        TunnelWriteThread.queue_lock.lock();
        TunnelWriteThread.write_queue.add(buffer);
        TunnelWriteThread.queue_lock.unlock();
    }

    private void dequeBuffer() {
        byte[] data = null;
        try {
            TunnelWriteThread.write_lock.lock();
            data = TunnelWriteThread.write_queue.poll();
            if (Objects.isNull(data)) {
                return;
            }
            writeBuffer(data);
        }
        catch (IOException e) {
            e.printStackTrace();
            if (Objects.isNull(data)) {
                System.out.println("Failed while writing data: " + TunnelUtil.packetToString(data));
            }
            else {
                System.out.println("Failed while writing uninitialized data");
            }
            client.setIsOpen(false);
        }
        finally {
            TunnelWriteThread.write_lock.unlock();
        }
    }
}
