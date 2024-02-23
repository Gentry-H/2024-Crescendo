package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

import java.util.Objects;

/**
 * This class automatically generates the autonomous routine based on six inputs
 */
public class AutonomousCommandsBuilder extends SequentialCommandGroup implements Constants.AutoConstants, Constants.PathPlannerConstants {

    /**
     * This class automatically generates the autonomous routine based on six inputs
     * @param numOfNotesToScore {@link Integer} The total number of notes to score
     * @param numOfAmpScores {@link Integer} The number of notes to score in the amp
     * @param autoLane {@link String} The general area of the field the robot should drive in. MUST BE DEFINED IN CONSTANTS (see {@link frc.robot.Constants.AutoConstants}
     * @param numOfNotesFromStart {@link Integer} The number of notes to pickup from the starting area
     * @param searchDirection {@link Integer} Which direction the robot should search in (currently 1 (toward source) or -1 (towards amp), may change)
     * @param grabFromCenterFirst {@link Boolean} Whether to grab
     */
    public AutonomousCommandsBuilder(int numOfNotesToScore, int numOfAmpScores, String autoLane,
                                     int numOfNotesFromStart, String searchDirection, boolean grabFromCenterFirst,
                                     NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive){


        /*
         * Pretty much everywhere you see a "Commands.print('Something in here')" it's just a
         * temporary command for testing that just prints out the action it's supposed to do so that
         * we can see that it's choosing all the right actions in the right order.
         * Everywhere there's one of those there should also be a line or method that's commented out that would
         * call the action we actually want the robot to do, so when this is all finished we just remove the
         * Commands.print() statements and uncomment the other ones.
         *
         * Everywhere there's a System.out.println() statement is where we are just printing out what action
         * is being scheduled when it's scheduled, before any of them actually start, so that way we can see
         * what order they were called in, and that they all ran in that order
         */

        // Make sure we don't try to take more notes from the start than we want to score
        if(numOfNotesFromStart >= numOfNotesToScore) numOfNotesFromStart = numOfNotesToScore-1;

        // Calculate the number of notes the robot should get from the center
        int numOfNotesFromCenter = numOfNotesToScore-(numOfNotesFromStart+1);

        // The order we should grab the start notes in
        int[] orderOfStartNotes = orderOfNotes(numOfNotesFromStart, autoLane, searchDirection, 3);
        // The order we should grab the center notes in
        int[] orderOfCenterNotes = orderOfNotes(numOfNotesFromCenter, autoLane, searchDirection, 5);

        // Keep track of the number of grabs from the start we have attempted
        int grabsFromStartAttempted = 0;
        // Keep track of where we last scored
        String lastScoredIn;

        if (numOfNotesToScore > 0) {
            // Shoot in the speaker firsts thing
            addCommands(
                    Commands.print("Shooting Into Speaker!"),
                    notePlayerSubsystem.aimAndSpinUpForSpeaker().withTimeout(0.5),
                    notePlayerSubsystem.aimAndSpinUpForSpeaker().until(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY).withTimeout(1.5),
                    //.until(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)
                    notePlayerSubsystem.aimAndSpinUpForSpeaker().alongWith(notePlayerSubsystem.scoreNote()),
                    notePlayerSubsystem.prepForPickup()
            );
            System.out.println("Shoot Speaker");
            lastScoredIn = "Sp";

            if (numOfNotesToScore == 1) {
                addCommands(
                        Commands.print("Only 1 Action Chosen! Leaving Starting Zone!"),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-SL"))
                        //Commands.print("Following: " + lastScoredIn + "-" + autoLane + "-SL")
                );
            }
            else {
                for (int i = 0; i < numOfNotesToScore - 1; i++) {

                    // We should get from the center line if we are getting from the center line first,
                    // and we haven't already gotten all the notes from the center line that we planned to
                    // OR if we are grabbing from the start first, but we've already gotten all the ones from start that we planned to
                    if (((grabFromCenterFirst && i < numOfNotesFromCenter) || (!grabFromCenterFirst && i >= numOfNotesFromStart))) {
                        System.out.println("Grab From Center");
                        addCommands(
                                Commands.print("Grabbing From Center!"),
                                notePlayerSubsystem.prepForPickup(),
                                notePlayerSubsystem.driveArmPercent(() -> -0.125).until(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY),
                                new AutoGrabFromCenter(orderOfCenterNotes, lastScoredIn, autoLane, notePlayerSubsystem, swerveDrive)
                                        // Unless all the center notes we wanted are gone
                                        .unless(() -> FieldState.getInstance().isCenterNotesGone())
                        );

                        // Prioritize scoring in the Amp (not sure if we want it this way)
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            addCommands(
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Amp"),
                                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Amp"), pathConstraints),
                                    notePlayerSubsystem.prepForAmp(),
                                    notePlayerSubsystem.driveArmPercent(() -> 0.15).raceWith(Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)),
                                    notePlayerSubsystem.scoreAmp(),
                                    notePlayerSubsystem.prepForPickup()
                            );
                        } else {
                            // Score in the Speaker
                            lastScoredIn = "Sp";
                            addCommands(
                                    // TODO: add command for calculating direction to face and shooter velocity, and spin up the shooter, probably as it follows the path
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Sp"),
                                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Sp"), pathConstraints),
                                    notePlayerSubsystem.aimAndSpinUpForSpeaker()
                                            .alongWith(notePlayerSubsystem.driveArmPercent(() -> 0.15)).withTimeout(2)
                                            //.until(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)
                                            .andThen(notePlayerSubsystem.feedNoteToShooter().raceWith(notePlayerSubsystem.aimAndSpinUpForSpeaker()).andThen(notePlayerSubsystem.finishShooting())),
                                    notePlayerSubsystem.prepForPickup()
                            );
                        }
                    } else {
                        // If we aren't supposed to grab from the center, then grab from the start
                        System.out.println("Grabbing From Start");
                        addCommands(
                                Commands.print("Grabbing from start!"),
                                notePlayerSubsystem.prepForPickup(),
                                notePlayerSubsystem.driveArmPercent(() -> -0.125).raceWith(Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)),
                                new AutoGrabFromStart(orderOfStartNotes[grabsFromStartAttempted], lastScoredIn, autoLane, notePlayerSubsystem, swerveDrive)
                        );

                        // Prioritize scoring in the Amp (not sure if we want it this way)
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            System.out.println("Shoot Amp");
                            addCommands(
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Amp"),
                                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Amp"), pathConstraints)
                                            .alongWith(notePlayerSubsystem.positionNote()),
                                    notePlayerSubsystem.prepForAmp(),
                                    notePlayerSubsystem.driveArmPercent(() -> 0.15).raceWith(Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)),
                                    notePlayerSubsystem.scoreAmp(),
                                    notePlayerSubsystem.prepForPickup()
                            );
                        } else {
                            // Score in the Speaker
                            lastScoredIn = "Sp";
                            System.out.println("Shoot Speaker");
                            addCommands(
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Sp"),
                                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Sp"), pathConstraints),
                                    notePlayerSubsystem.aimAndSpinUpForSpeaker()
                                            .alongWith(notePlayerSubsystem.driveArmPercent(() -> 0.15))
                                            .until(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)
                                            .andThen(notePlayerSubsystem.feedNoteToShooter().raceWith(notePlayerSubsystem.aimAndSpinUpForSpeaker()).andThen(notePlayerSubsystem.finishShooting())),
                                    notePlayerSubsystem.prepForPickup()
                            );
                        }
                        // Keep track of how many times we have tried to get a note from the center
                        grabsFromStartAttempted++;
                    }
                }
                addCommands(
                        Commands.print("Done Scoring! Driving to Center Line!"),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-CL"), pathConstraints),
                        Commands.print("Autonomous Routine is Complete!")
                );
            }
        }
        addCommands(Commands.none());
    }

    /**
     * Get an array containing the number of each note, in the order we want to get them
     * @param numOfNotesToGet The number of notes to get (the length of the array)
     * @param autoLane The autonomous lane
     * @param searchDirection The direction to look for notes in if we are going under the stage
     * @param totalNumOfNotes Then total number of notes available (3 for starting notes, 5 for center line notes)
     * @return An array containing hte number of each note to get, in the order to get them ([0] contains the number of the first note to get)
     */
    private int[] orderOfNotes(int numOfNotesToGet, String autoLane, String searchDirection, int totalNumOfNotes) {

        // Don't do anything if the number of notes to get is out of bounds
        if(numOfNotesToGet < 1) return  new int[0];
        else if(numOfNotesToGet > totalNumOfNotes) return new int[0];

        int[] order = new int[numOfNotesToGet];

        // If we are using the Amp Side Lane, get the notes in ascending order (1-5), note #1 is the one closes to the Amp
        if(Objects.equals(autoLane, ampLane)) {
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = i+1;
            }
        }

        // If we are using the Source Side Lane, get the notes in descending order (5-1), note #5 is the one closest to the source
        else if(Objects.equals(autoLane, sourceLane)) {
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = totalNumOfNotes-i;
            }
        }

        // If we are using the Under-Stage Lane, start with the middle one, and work our way up, down,
        // or from the middle outward, depending on the searchDirection
        else if (Objects.equals(autoLane, stageLane)) {
            // find the center note
            int noteNumber = (int) Math.ceil((double) totalNumOfNotes / 2);
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = noteNumber;

                if (Objects.equals(searchDirection, "ToAmp")) { // Do we want to go toward the Amp?
                    noteNumber--;
                } else if (Objects.equals(searchDirection, "ToSrc")) { // Do we want to go toward the Source?
                    noteNumber++;
                } else { // Otherwise go from the middle outward
                    if (i%2 == 0) {
                        noteNumber += i+1;
                    } else {
                        noteNumber -= i+1;
                    }
                }

                // If the note number is out of bounds, reset it to the total number of notes
                // In the case that we are grabbing all 5 center notes, and the search direction is
                // -1 (toward the Amp) we would start with 3, then go to 2, and then 1, and the next would be 0,
                // but since that doesn't exist we would instead roll over to 5, and then go to 4
                if(noteNumber < 1) noteNumber += totalNumOfNotes;
                else if(noteNumber > totalNumOfNotes) noteNumber -= totalNumOfNotes;
            }
        }

        return order;
    }
}