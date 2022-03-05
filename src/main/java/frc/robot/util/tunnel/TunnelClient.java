package frc.robot.util.tunnel;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.Objects;


public class TunnelClient {
    public Socket socket;
    public TunnelInterface tunnel_interface;
    public TunnelProtocol protocol;

    public InputStream input = null;
    public OutputStream output = null;

    private TunnelReadThread read_thread;
    private TunnelWriteThread write_thread;

    private boolean shouldClose = false;
    
    public TunnelClient(TunnelInterface tunnel_interface, Socket clientSocket) {
        this.tunnel_interface = tunnel_interface;

        init(clientSocket);
    }

    public boolean isActive() {
        return read_thread.isAlive() && write_thread.isAlive();
    }

    public void writePacket(String category, Object... objects) {
        byte[] data = protocol.makePacket(category, objects);
        write_thread.queueBuffer(data);
    }

    public boolean getShouldClose() {
        return shouldClose;
    }

    public void setShouldClose(boolean shouldClose) {
        this.shouldClose = shouldClose;
    }
    public void close() {
        if (Objects.isNull(input) || Objects.isNull(output) || Objects.isNull(socket)) {
            return;
        }
        try {
            System.out.println("Closing tunnel client");
            input.close();
            output.close();
            socket.close();
            shouldClose = true;
        }
        catch (IOException e) {
            System.out.println("Failed while attempting to close client: " + e.getMessage());
        }
    }

    public void init(Socket clientSocket) {
        this.socket = clientSocket;
        protocol = new TunnelProtocol();

        System.out.println("Opening client");
        try {
            input = this.socket.getInputStream();
            output = this.socket.getOutputStream();
            shouldClose = false;
        }
        catch (IOException e) {
            shouldClose = true;
            System.out.println("Failed to open client. " + e.getMessage());
        }

        read_thread = new TunnelReadThread(this);
        write_thread = new TunnelWriteThread(this);

        read_thread.start();
        write_thread.start();
    }
}
