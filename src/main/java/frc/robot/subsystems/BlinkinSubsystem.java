package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.lib.bluecrew.util.BlinkinValues;
import frc.robot.Constants;

import frc.lib.bluecrew.util.RobotState;

public class BlinkinSubsystem extends SubsystemBase  implements Constants.Misc, Constants.GameStateConstants, BlinkinValues {

    private final Spark blinkinOutput;

    RobotState rs = RobotState.getInstance();


    /**
     * The Singleton instance of this BlinkinSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static BlinkinSubsystem INSTANCE = new BlinkinSubsystem();

    /**
     * Returns the Singleton instance of this BlinkinSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code BlinkinSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static BlinkinSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this BlinkinSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private BlinkinSubsystem() {
        blinkinOutput = new Spark(BLINKIN_PORT);
    }

    @Override
    public void periodic() {
        switch (rs.getShooterMode()) {
            case SPEAKER -> {
                switch (rs.getRobotCycleStatus()) {
                    case NO_NOTE_CANT_SEE_SPEAKER ->
                        blinkinOutput.set(RED);
                    case NO_NOTE_SEES_SPEAKER ->
                        blinkinOutput.set(RED_ORANGE);
                    case HAS_NOTE_CANT_SEE_SPEAKER ->
                        blinkinOutput.set(YELLOW);
                    case HAS_NOTE_SEES_SPEAKER -> {
                        switch (rs.getShooterStatus()) {
                            case READY ->
                                blinkinOutput.set(DARK_GREEN);
                            case UNREADY ->
                                blinkinOutput.set(BLUE_GREEN);
                        }
                    }
                }
            }
            case PICKUP -> {
                if (!rs.hasNote()) {
                    if (rs.isNoteIsAvailable()) {
                        blinkinOutput.set(HEARTBEAT_BLUE);
                    } else {
                        blinkinOutput.set(HEARTBEAT_RED);
                    }
                } else blinkinOutput.set(LAWN_GREEN);
            }
            case AMP -> {
                if (rs.hasNote()) {
                    switch (rs.getShooterStatus()) {
                        case READY -> blinkinOutput.set(DARK_GREEN);
                        case UNREADY -> blinkinOutput.set(BLUE_GREEN);
                    }
                } else blinkinOutput.set(RED_ORANGE);
            }
        }
    }

    public void setColorMode(double mode) {
        blinkinOutput.set(mode);
    }
}

