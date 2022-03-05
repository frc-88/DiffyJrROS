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
    private TunnelDataRelayThread data_relay_thread;

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms, boolean auto_start)
    {
        if (!Objects.isNull(instance)) {
            throw new RuntimeException("Only once instance of TunnelServer allowed");
        }
        instance = this;

        this.port = port;
        this.tunnel_interface = tunnel_interface;
        this.data_relay_thread = new TunnelDataRelayThread(tunnel_interface, data_relay_delay_ms);

        if (auto_start) {
            this.start();
        }
    }

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms) {
        this(tunnel_interface, port, data_relay_delay_ms, true);
    }

    public boolean anyClientsAlive()
    {
        if (Objects.isNull(client)) {
            return false;
        }
        boolean isActive = client.isActive();
        return isActive;
    }

    // Write a packet to all connected clients
    public void writePacket(String category, Object... objects)
    {
        if (Objects.nonNull(client) && client.isActive()) {
            client.writePacket(category, objects);
        }
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
            this.data_relay_thread.start();
            while (true) {
                Socket socket = null;
                try {
                    socket = serverSocket.accept();
                } catch (IOException e) {
                    System.out.println("I/O error: " + e);
                    continue;
                }

                if (Objects.isNull(client)) {
                    // new threads for a client
                    client = new TunnelClient(tunnel_interface, socket);
                }
                else {
                    client.close();
                    Thread.sleep(50);
                    client.init(socket);
                }
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
