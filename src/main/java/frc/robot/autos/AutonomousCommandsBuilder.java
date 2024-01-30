package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.bluecrew.util.GlobalVariables;
import frc.robot.commands.ShootNoteIntoAmp;
import frc.robot.commands.ShootNoteIntoSpeaker;

import java.util.Objects;

import static frc.robot.Constants.AutoConstants.*;

/**
 * This class automatically generates the autonomous routine based on six inputs
 */
public class AutonomousCommandsBuilder extends SequentialCommandGroup {

    /**
     * This class automatically generates the autonomous routine based on six inputs
     * @param numOfAutoActions {@link Integer} The number of notes to score
     * @param numOfAmpScores {@link Integer} The number of notes to score in the amp
     * @param autoLane {@link String} The general area of the field the robot should drive in. MUST BE DEFINED IN CONSTANTS (see {@link frc.robot.Constants.AutoConstants}
     * @param numOfNotesFromStart {@link Integer} The number of notes to pickup from the starting area
     * @param searchDirection {@link Integer} Which direction the robot should search in (currently 1 (toward source) or -1 (towards amp), may change)
     * @param grabFromCenterFirst {@link Boolean} Whether to grab
     */
    public AutonomousCommandsBuilder(int numOfAutoActions, int numOfAmpScores, String autoLane,
                                     int numOfNotesFromStart, int searchDirection, boolean grabFromCenterFirst){


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
        if(numOfNotesFromStart >= numOfAutoActions) numOfNotesFromStart = numOfAutoActions-1;

        // Calculate the number of notes the robot should get from the center
        int numOfNotesFromCenter = numOfAutoActions-(numOfNotesFromStart+1);

        // The order we should grab the start notes in
        int[] orderOfStartNotes = orderOfNotes(numOfNotesFromStart, autoLane, searchDirection, 3);
        // The order we should grab the center notes in
        int[] orderOfCenterNotes = orderOfNotes(numOfNotesFromCenter, autoLane, searchDirection, 5);

        // Keep track of the number of grabs from the start we have attempted
        int grabsFromStartAttempted = 0;
        // Keep track of where we last scored
        String lastScoredIn;

        if (numOfAutoActions > 0) {
            // Shoot in the speaker firsts thing
            addCommands(/*new ShootNoteIntoSpeaker()*/Commands.print("ShootIntoSpeaker"));
            System.out.println("Shoot Speaker");
            lastScoredIn = "Sp";


            // Something here with this for loop and the proceeding if statement seems wrong,
            // I've been staring at this code for too long
            // Also there's and else statement somewhere that just drives out of the starting zone,
            // which should probably be moved to work properly, and also somewhere should be something
            // to always end by driving to the center line, in case we have enough left over time to do that,
            // so we can get another note more quickly at the start of teleop
            for (int i = 0; i < numOfAutoActions-1; i++) {
                // Do we want score more than just the note we start with?
                if(numOfAutoActions > 1) {
                    // We should get from the center line if we are getting from the center line first,
                    // and we haven't already gotten all the notes from the center line that we planned to
                    // OR if we are grabbing from the start first, but we've already gotten all the ones from start that we planned to
                    if (((grabFromCenterFirst && i < numOfNotesFromCenter) || (!grabFromCenterFirst && i >= numOfNotesFromStart))) {
                        System.out.println("Grab From Center");
                        addCommands(
                                new AutoGrabFromCenter(orderOfCenterNotes, lastScoredIn, autoLane)
                                        // Unless all the center notes we wanted are gone
                                        .unless(() -> GlobalVariables.getInstance().isCenterNotesGone())
                        );

                        // Prioritize scoring in the Amp (not sure if we want it this way)
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Amp"), pathConstraints),
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Amp"),
                                    new ShootNoteIntoAmp()
                                            // Only if we have a note
                                            .onlyIf(() -> GlobalVariables.getInstance().hasNote())
                            );
                        } else {
                            // Score in the Speaker
                            lastScoredIn = "Sp";
                            addCommands(
                                    // TODO: add command for calculating direction to face and shooter velocity, and spin up the shooter, probably or as it follows the path
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-Sp"), pathConstraints),
                                    Commands.print("Path Find To and Following: CL-" + autoLane + "-Sp"),
                                    new ShootNoteIntoSpeaker()
                                            // Only if we have a note
                                            .onlyIf(() -> GlobalVariables.getInstance().hasNote())
                            );
                        }
                    } else {
                        // If we aren't supposed to grab from the center, then grab from the start
                        System.out.println("Grabbing From Start");
                        addCommands(new AutoGrabFromStart(orderOfStartNotes[grabsFromStartAttempted], lastScoredIn, autoLane));

                        // Prioritize scoring in the Amp (not sure if we want it this way)
                        if (i < numOfAmpScores) {
                            lastScoredIn = "Amp";
                            System.out.println("Shoot Amp");
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Amp"), pathConstraints),
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Amp"),
                                    new ShootNoteIntoAmp()
                                            // Only if we have a note
                                            .onlyIf(() -> GlobalVariables.getInstance().hasNote())
                            );
                        } else {
                            // Score in the Speaker
                            lastScoredIn = "Sp";
                            System.out.println("Shoot Speaker");
                            addCommands(
                                    //AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("SL-" + autoLane + "-Sp"), pathConstraints),
                                    Commands.print("Path Find To and Following: SL-" + autoLane + "-Sp"),
                                    new ShootNoteIntoSpeaker()
                                            // Only if we have a note
                                            .onlyIf(() -> GlobalVariables.getInstance().hasNote())
                            );
                        }

                        // Keep track of how many times we have tried to get a note from the center
                        grabsFromStartAttempted++;
                    }
                } else {
                    // If we're just scoring the note we start with, drive out of the starting zone, so we get the mobility points
                    addCommands(
                            //AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-SL"))
                            Commands.print("Following: " + lastScoredIn + "-" + autoLane + "-SL")
                    );
                }
            }
        }
    }

    /**
     * Get an array containing the number of each note, in the order we want to get them
     * @param numOfNotesToGet The number of notes to get (the length of the array)
     * @param autoLane The autonomous lane
     * @param searchDirection The direction to look for notes in if we are going under the stage
     * @param totalNumOfNotes Then total number of notes available (3 for starting notes, 5 for center line notes)
     * @return An array containing hte number of each note to get, in the order to get them ([0] contains the number of the first note to get)
     */
    private int[] orderOfNotes(int numOfNotesToGet, String autoLane, int searchDirection, int totalNumOfNotes) {

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

        // If we are using the Under-Stage Lane, start with the middle one, and work our way up or down
        // depending on the searchDirection (which is currently either -1 or 1)
        // TODO: add an option for getting the ones closest the the middle first
        else if (Objects.equals(autoLane, stageLane)) {
            // find the center note
            int noteNumber = (int) Math.ceil((double) totalNumOfNotes / 2);
            for(int i = 0; i < numOfNotesToGet; i++) {
                order[i] = noteNumber;

                // Find the note in the direction that we want
                noteNumber += searchDirection;

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