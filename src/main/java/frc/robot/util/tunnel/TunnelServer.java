package frc.robot.util.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Objects;
import java.util.concurrent.locks.ReentrantLock;

public class TunnelServer extends Thread {

    private int port = 0;
    private TunnelInterface tunnel_interface;
    private TunnelClient client;

    public static TunnelServer instance = null;
    private static ReentrantLock client_lock = new ReentrantLock();

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms, boolean auto_start)
    {
        if (!Objects.isNull(instance)) {
            throw new RuntimeException("Only once instance of TunnelServer allowed");
        }
        instance = this;

        this.port = port;
        this.tunnel_interface = tunnel_interface;

        if (auto_start) {
            this.start();
        }
    }

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms) {
        this(tunnel_interface, port, data_relay_delay_ms, true);
    }

    public boolean anyClientsAlive()
    {
        client_lock.lock();
        if (Objects.isNull(client)) {
            return false;
        }
        boolean isActive = client.isActive();
        client_lock.unlock();
        return isActive;
    }

    // Write a packet to all connected clients
    public void writePacket(String category, Object... objects)
    {
        client_lock.lock();
        if (Objects.nonNull(client) && client.isActive()) {
            client.writePacket(category, objects);
        }
        client_lock.unlock();
    }

    // Send message to all clients
    public void println(String message) {
        writePacket("__msg__", message);
    }

    private void loop() {
        ServerSocket serverSocket = null;

        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            System.out.println("IOException encountered a new socket: " + e.getMessage());
            return;
        }
        System.out.println("Socket is open");
        try
        {
            while (true) {
                Socket socket = null;
                try {
                    socket = serverSocket.accept();
                } catch (IOException e) {
                    System.out.println("I/O error: " + e);
                    continue;
                }

                client_lock.lock();
                if (Objects.isNull(client)) {
                    // new threads for a client
                    client = new TunnelClient(tunnel_interface, socket);
                }
                else {
                    client.close();
                    Thread.sleep(50);
                    client.init(socket);
                }
                client_lock.unlock();
            }
        }
        catch (InterruptedException e) {}
        finally {
            try {
                System.out.println("Closing socket");
                serverSocket.close();
            } catch (IOException e) {
                System.out.println("IOException encountered closing socket: " + e.getMessage());
            }
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException e) {}
        }
    }

    @Override
    public void run()
    {
        while (true) {
            loop();
        }
    }
}
