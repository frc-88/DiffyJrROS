package frc.robot.util.tunnel;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;


public class TunnelClient {
    public Socket socket;
    public TunnelInterface tunnel_interface;
    public TunnelProtocol protocol;

    public InputStream input = null;
    public OutputStream output = null;
    private boolean isOpen = false;
    private boolean shouldCloseThreads = false;

    private TunnelReadThread read_thread;
    private TunnelWriteThread write_thread;
    
    public TunnelClient(TunnelInterface tunnel_interface, Socket clientSocket) {
        this.socket = clientSocket;
        this.tunnel_interface = tunnel_interface;

        protocol = new TunnelProtocol();

        System.out.println("Opening client");
        try {
            input = socket.getInputStream();
            output = socket.getOutputStream();
            isOpen = true;

        } catch (IOException e) {
            e.printStackTrace();
            isOpen = false;
        }

        read_thread = new TunnelReadThread(this);
        write_thread = new TunnelWriteThread(this);
    }

    public void start() {
        read_thread.start();
        write_thread.start();
        shouldCloseThreads = false;
    }

    public boolean isAlive() {
        return read_thread.isAlive() && write_thread.isAlive();
    }

    public void writePacket(String category, Object... objects) {
        byte[] data = protocol.makePacket(category, objects);
        write_thread.queueBuffer(data);
    }
    public boolean isOpen() {
        return isOpen;
    }

    public boolean getShouldCloseThreads() {
        return shouldCloseThreads;
    }

    public void setIsOpen(boolean isOpen) {
        this.isOpen = isOpen;
    }

    public void close() {
        try {
            System.out.println("Closing tunnel client");
            input.close();
            output.close();
            socket.close();
            shouldCloseThreads = true;
        }
        catch (IOException e) {
            e.printStackTrace();
            System.out.println("Failed while attempting to close client");
        }
    }

}
