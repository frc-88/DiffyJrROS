package frc.robot.driverinput;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InstantCommandAnyMode extends InstantCommand {
    public InstantCommandAnyMode(Runnable toRun) {
        super(toRun);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
