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
            if (client.getShouldClose()) {
                break;
            }

            byte[] data = null;
            try {
                TunnelWriteThread.write_lock.lock();
                data = TunnelWriteThread.write_queue.poll();
                if (Objects.isNull(data)) {
                    Thread.sleep(5);
                    continue;
                }
                if (!writeBuffer(data)) {
                    break;
                }
            }
            catch (InterruptedException e) {
                break;
            }
            catch (IOException e) {
                if (Objects.isNull(data)) {
                    System.out.println("Failed while writing data: " + TunnelUtil.packetToString(data) + ". " + e.getMessage());
                }
                else {
                    System.out.println("Failed while writing uninitialized data. " + e.getMessage());
                }
            }
            finally {
                TunnelWriteThread.write_lock.unlock();
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

    public void queueBuffer(byte[] buffer) {
        TunnelWriteThread.queue_lock.lock();
        TunnelWriteThread.write_queue.add(buffer);
        TunnelWriteThread.queue_lock.unlock();
    }
}
