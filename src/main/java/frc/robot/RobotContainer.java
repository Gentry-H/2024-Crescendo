package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.bluecrew.util.BlinkinValues;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.autos.AutonomousCommandsBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

import java.util.function.BooleanSupplier;

import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Constants.AutoConstants {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController auxDriver = new CommandXboxController(1);

    /* Driver Buttons */
    private final BooleanSupplier cancelAutoRotation = () -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1);

    /* Subsystems */
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final NotePlayerSubsystem notePlayerSubsystem = new NotePlayerSubsystem();

    // Sendable Choosers for autonomous
    // total number of notes to score (including in speaker+amp) during auto
    private SendableChooser<Integer> numOfNotesToScoreChooser;
    // number of notes to score in amp during auto
    private SendableChooser<Integer> numOfAmpScoresChooser;
    // number of notes to pick up from starting line area
    private SendableChooser<Integer> numOfNotesFromStartChooser;
    // should travel to-from center line go under stage, near amp, or near source
    private SendableChooser<String> autoLaneChooser;
    // if you travel under stage, do you go toward amp or source or go from the middle out to find notes
    private SendableChooser<String> directionToSearchInChooser;
    // prioritize getting notes from center line (over those from starting area)
    private SendableChooser<Boolean> grabFromCenterFirstChooser;

    private ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // PathPlanner command templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        NamedCommands.registerCommand("EndPathAction", Commands.print("End of the Path Action"));
        NamedCommands.registerCommand("EndNoteAction", Commands.print("End of the Note Action"));

/*
        Thread visionThread = new Thread(new VisionPipelineRunnable(VisionModule.getInstance()), "visionThread");
        visionThread.setDaemon(true);
        visionThread.start();
*/
        setupAutoChoosers();

        // Fire up the blinkin
        BlinkinSubsystem.getInstance().setColorMode(BlinkinValues.BLUE);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default Commands:
        swerveDrive.setDefaultCommand(
                swerveDrive.teleopDriveSwerveDriveCommand(
                        driver::getLeftY,
                        driver::getLeftX,
                        driver::getRightTriggerAxis,
                        driver::getRightX,
                        () -> false//driver.leftBumper().getAsBoolean()
                ));

        notePlayerSubsystem.setDefaultCommand(notePlayerSubsystem.allStop());

//        /* Driver Buttons */
//        driver.povCenter().onFalse(Commands.waitSeconds(0.1).andThen(swerveDrive.setHoldHeading(-driver.getHID().getPOV()).until(cancelAutoRotation)));

        driver.rightTrigger(0.75).onTrue(swerveDrive.faceSpeaker().until(cancelAutoRotation));

        driver.x().onTrue(new InstantCommand(swerveDrive::xLockWheels));
        driver.start().whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("AlignAmp"), Constants.PathPlannerConstants.pathConstraints));

        driver.rightBumper().whileTrue((new FindAndGotoNote(notePlayerSubsystem, swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake))
                .alongWith(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).andThen(notePlayerSubsystem.intakeNote())));
//        driver.leftBumper().onTrue(notePlayerSubsystem.feedNoteToShooter().andThen(notePlayerSubsystem.finishShooting()));
//        driver.a().onTrue(notePlayerSubsystem.scoreAmp());
        driver.leftBumper().onTrue(notePlayerSubsystem.scoreNote());

        auxDriver.leftBumper().whileTrue(notePlayerSubsystem.driveArmPercent(() -> 0.15));
        auxDriver.rightBumper().whileTrue(notePlayerSubsystem.driveArmPercent(() -> -0.125));
        auxDriver.b().whileTrue(notePlayerSubsystem.aimAndSpinUpForSpeaker());
        auxDriver.a().onTrue(notePlayerSubsystem.prepForPickup());
        auxDriver.y().onTrue(notePlayerSubsystem.prepForAmp());
        auxDriver.x().whileTrue(notePlayerSubsystem.eject());

//        auxDriver.y().onTrue(notePlayerSubsystem.rotateArmToDegrees(-10));
//        auxDriver.b().onTrue(notePlayerSubsystem.rotateArmToDegrees(-20));
//        auxDriver.x().onTrue(notePlayerSubsystem.rotateArmToDegrees(-36));
//        auxDriver.a().onTrue(notePlayerSubsystem.rotateArmToDegrees(10));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AutonomousCommandsBuilder(numOfNotesToScoreChooser.getSelected(), numOfAmpScoresChooser.getSelected(),
                autoLaneChooser.getSelected(), numOfNotesFromStartChooser.getSelected(),
                directionToSearchInChooser.getSelected(), grabFromCenterFirstChooser.getSelected(),
                notePlayerSubsystem, swerveDrive);
        //return Commands.none();//notePlayerSubsystem.aimAtTarget().repeatedly();
    }

    /**
     * Creates each {@link SendableChooser} for Autonomous
     */
    public void setupAutoChoosers() {
        // Chooser for number of actions in auto
        numOfNotesToScoreChooser = new SendableChooser<>();
        numOfNotesToScoreChooser.setDefaultOption("0", 0);
        for (int i = 1; i <= 5; i++) {
            numOfNotesToScoreChooser.addOption("" + i, i);
        }

        // Choose how many notes to score in Amp
        numOfAmpScoresChooser = new SendableChooser<>();
        numOfAmpScoresChooser.setDefaultOption("0", 0);
        numOfAmpScoresChooser.addOption("1", 1);
        numOfAmpScoresChooser.addOption("2", 2);

        // Choose which lane the robot should travel in
        autoLaneChooser = new SendableChooser<>();
        autoLaneChooser.setDefaultOption("None", null);
        autoLaneChooser.addOption("AmpSideLane", ampLane);
        autoLaneChooser.addOption("UnderStageLane", stageLane);
        autoLaneChooser.addOption("SourceSideLane", sourceLane);

        // Choose how many notes to get from the starting zone
        numOfNotesFromStartChooser = new SendableChooser<>();
        numOfNotesFromStartChooser.setDefaultOption("0", 0);
        numOfNotesFromStartChooser.addOption("1", 1);
        numOfNotesFromStartChooser.addOption("2", 2);
        numOfNotesFromStartChooser.addOption("3", 3);

        // Choose Which direction the robot will search for notes in
        directionToSearchInChooser = new SendableChooser<>();
        directionToSearchInChooser.setDefaultOption("TowardsAmp", "ToAmp");
        directionToSearchInChooser.addOption("TowardsSource", "ToSrc");
        directionToSearchInChooser.addOption("MiddleOut", "MidOut");

        // Choose whether to grab notes from the center or the start first
        grabFromCenterFirstChooser = new SendableChooser<>();
        grabFromCenterFirstChooser.setDefaultOption("GrabFromCenterFirst", true);
        grabFromCenterFirstChooser.addOption("GrabFromStartFirst", false);

        autonomousTab.add("Number Of Auto Actions", numOfNotesToScoreChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Number Of Amp Scores", numOfAmpScoresChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Autonomous Lane", autoLaneChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Number Of Notes From Start", numOfNotesFromStartChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Direction To Search In", directionToSearchInChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Grab From Where", grabFromCenterFirstChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
    }
}
