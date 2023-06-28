package frc.robot.ros.bridge;

import java.util.Optional;

public interface Subscriber<T> {
    public Optional<T> receive();
}
