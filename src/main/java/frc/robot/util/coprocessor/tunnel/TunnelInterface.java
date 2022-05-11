package frc.robot.util.coprocessor.tunnel;

public interface TunnelInterface {
    public void packetCallback(DataStreamInterface data_stream, PacketResult result);
    public void update();
}
