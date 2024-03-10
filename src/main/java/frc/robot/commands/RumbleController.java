package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleController extends Command {

    private final GenericHID controller;
    private final GenericHID.RumbleType rumbleType;

    private final double rumbleStrength;
    private final double rumbleTime;

    private final Timer timer = new Timer();

    public RumbleController(GenericHID controller, GenericHID.RumbleType rumbleType, double rumbleStrength, double rumbleTime) {
        this.controller = controller;
        this.rumbleType = rumbleType;

        this.rumbleStrength = rumbleStrength;
        this.rumbleTime = rumbleTime;
    }

    public RumbleController(GenericHID controller, GenericHID.RumbleType rumbleType, double rumbleTime) {
        this.controller = controller;
        this.rumbleType = rumbleType;

        this.rumbleTime = rumbleTime;
        this.rumbleStrength = 1;
    }

    public RumbleController(GenericHID controller, double rumbleTime) {
        this.controller = controller;
        this.rumbleTime = rumbleTime;

        this.rumbleStrength = 1;
        this.rumbleType = GenericHID.RumbleType.kBothRumble;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (RobotState.isTeleop()) {
            controller.setRumble(rumbleType, rumbleStrength);
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(rumbleType, 0);

        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(rumbleTime);
    }
}
