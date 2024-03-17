package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

import java.util.Objects;

import static frc.robot.Constants.GameStateConstants.ShooterStatus.READY;

/**
 * This class automatically generates the autonomous routine based on six inputs
 */
public class AutonomousCommandsBuilder extends SequentialCommandGroup implements Constants.AutoConstants, Constants.PathPlannerConstants {

    /**
     * This class automatically generates the autonomous routine based on six inputs
     * @param numOfNotesToScore {@link Integer} The total number of notes to score
     * @param autoLane {@link String} The general area of the field the robot should drive in. MUST BE DEFINED IN CONSTANTS (see {@link frc.robot.Constants.AutoConstants}
     * @param numOfNotesFromStart {@link Integer} The number of notes to pickup from the starting area
     * @param searchDirection {@link Integer} Which direction the robot should search in (currently 1 (toward source) or -1 (towards amp), may change)
     * @param grabFromCenterFirst {@link Boolean} Whether to grab
     */
    public AutonomousCommandsBuilder(int numOfNotesToScore, String autoLane,
                                     int numOfNotesFromStart, String searchDirection, boolean grabFromCenterFirst,
                                     NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive, double delay) {

//        DataLogManager.log("*********************************************************************** Starting Building Auto! ***************************************************************************************");
//        long startTime = System.nanoTime();

        if (autoLane != null) {

            // Make sure we don't try to take more notes from the start than we want to score
            if (numOfNotesFromStart >= numOfNotesToScore) numOfNotesFromStart = numOfNotesToScore - 1;

            // Calculate the number of notes the robot should get from the center
            int numOfNotesFromCenter = numOfNotesToScore - (numOfNotesFromStart + 1);

            // The order we should grab the start notes in
            int[] orderOfStartNotes = orderOfNotes(numOfNotesFromStart, autoLane, searchDirection, 3);
            // The order we should grab the center notes in
            int[] orderOfCenterNotes = orderOfNotes(numOfNotesFromCenter, autoLane, searchDirection, 5);

            // Keep track of the number of grabs from the start we have attempted
            int grabsFromStartAttempted = 0;
            // Keep track of where we last scored
            String lastScoredIn;

            if (numOfNotesToScore > 0) {
                // Shoot in the speaker first thing
                addCommands(
//                    new InstantCommand(() -> DataLogManager.log("Shooting Into Speaker!")),
                        // TODO: add logic for shooting while moving to the next note to pick up
                        // Score in the speaker and then stop the shooter and indexer
                        new AutoScoreInSpeaker(notePlayerSubsystem).finallyDo(() -> {
//                            notePlayerSubsystem.getShooter().stop();
                            notePlayerSubsystem.getIndexer().stop();
//                            notePlayerSubsystem.shootFromSubwoofer();
                        }),
//                    new AutoLog("Finished Shooting Into Speaker!"),
                        // Make sure we're not trying to face the speaker while we aren't scoring
                        new InstantCommand(() -> {
                            swerveDrive.setFaceSpeaker(false);
//                            notePlayerSubsystem.shootFromSubwoofer();
                        }),
                        notePlayerSubsystem.prepForPickup(),
                        new InstantCommand(() -> swerveDrive.setShouldUseVision(true)),
                        Commands.waitSeconds(delay)
                );
//            DataLogManager.log("Shoot Speaker");
                lastScoredIn = "Sp";

                if (numOfNotesToScore == 1) {
                    addCommands(
                            // If we've selected to only score one note, just drive out of the starting zone
//                        new InstantCommand(() -> DataLogManager.log("Only 1 Action Chosen! Leaving Starting Zone!")),
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-SL"))
                            //new InstantCommand(() -> DataLogManager.log("Following: " + lastScoredIn + "-" + autoLane + "-SL")
                    );
                } else {
                    for (int i = 0; i < numOfNotesToScore - 1; i++) {

                        // We should get from the center line if we are getting from the center line first,
                        // and we haven't already gotten all the notes from the center line that we planned to
                        // OR if we are grabbing from the start first, but we've already gotten all the ones from start that we planned to
                        if (((grabFromCenterFirst && i < numOfNotesFromCenter) || (!grabFromCenterFirst && i >= numOfNotesFromStart))) {
//                        DataLogManager.log("Grab From Center");
                            addCommands(
//                                new InstantCommand(() -> DataLogManager.log("Grabbing From Center!")),
                                    // Prepare for pickup
                                    notePlayerSubsystem.prepForPickup(),
                                    // Grab one of the center notes based on the generated order to grab them in, and which ones we think still exist
                                    new AutoGrabFromCenter(orderOfCenterNotes, lastScoredIn, autoLane, notePlayerSubsystem, swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake)
                                            // Unless all the center notes we wanted are gone
                                            .unless(() -> FieldState.getInstance().isCenterNotesGone())
                            );
                             // Score in the Speaker
                            lastScoredIn = "Sp";
                            PathPlannerPath pathToSpeaker = PathPlannerPath.fromPathFile("CL-" + autoLane + "-Sp");
                            double distanceToSpeakerAtEndOfPath = pathToSpeaker.getAllPathPoints().get(pathToSpeaker.getAllPathPoints().size() - 1).position.getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d());
//                            addCommands(new InstantCommand(() -> DataLogManager.log("Path Find To and Following: CL-" + autoLane + "-Sp")));
                            if (!Objects.equals(autoLane, stageLane)) {
                                addCommands(
//                                        new InstantCommand(() -> swerveDrive.setFaceSpeaker(true)),
                                        AutoBuilder.buildAuto("CL-" + autoLane + "-Sp"),
                                        new AutoScoreInSpeaker(notePlayerSubsystem).finallyDo(() -> {
                                            notePlayerSubsystem.getIndexer().stop();
//                                                notePlayerSubsystem.getShooter().stop();
                                        })
                                );
                            } else {
                                addCommands(
                                        new InstantCommand(() -> notePlayerSubsystem.setMoveArmInAuto(true)),
                                        AutoBuilder.buildAuto("CL-" + autoLane + "-Sp")
                                );
                            }

                            addCommands(
                                    new InstantCommand(() -> RobotState.getInstance().setShooterMode(Constants.GameStateConstants.ShooterMode.SPEAKER)),
                                    new RunCommand(() -> notePlayerSubsystem.getShooter().spinMetersPerSecond(13))
                                            .alongWith(Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == READY)
                                                    .andThen(notePlayerSubsystem.scoreNote())),
                                    notePlayerSubsystem.prepForPickup(),
                                    new InstantCommand(() -> {
                                        swerveDrive.setFaceSpeaker(false);
                                    })
                            );
                        } else {
                            // If we aren't supposed to grab from the center, then grab from the start
//                        DataLogManager.log("Grabbing From Start");
                            addCommands(
//                                new InstantCommand(() -> DataLogManager.log("Grabbing from start!")),
                                    notePlayerSubsystem.prepForPickup(),
                                    new AutoGrabFromStart(orderOfStartNotes[grabsFromStartAttempted], lastScoredIn, autoLane, notePlayerSubsystem, swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake)
                            );
                            // Score in the Speaker
                            lastScoredIn = "Sp";
//                            DataLogManager.log("Shoot Speaker");
                            String pathName = "SN" + orderOfStartNotes[grabsFromStartAttempted] + "-" + autoLane + "-Sp";
                            PathPlannerPath pathToSpeaker = PathPlannerPath.fromPathFile(pathName);
                            double distanceToSpeakerAtEndOfPath = pathToSpeaker.getAllPathPoints().get(pathToSpeaker.getAllPathPoints().size() - 1)
                                    .position.getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d());
                            addCommands(
                                    new InstantCommand(() -> notePlayerSubsystem.setMoveArmInAuto(true)),
                                    AutoBuilder.buildAuto(pathName),
                                    //                                            new AutoLog("Finished intaking and following path"),
                                    new RunCommand(() -> notePlayerSubsystem.getIndexer().spin(0.65))
                                            .until(notePlayerSubsystem.getIndexer()::noteInIndexer)
                                            .unless(notePlayerSubsystem.getIndexer()::noteInIndexer)
                                            .andThen(notePlayerSubsystem.rotateArmToDegrees(notePlayerSubsystem.getAngleInterpolator().get(1.4d))),
                                    new AutoScoreInSpeaker(notePlayerSubsystem).finallyDo(() -> {
//                                            notePlayerSubsystem.getShooter().stop();
                                        notePlayerSubsystem.getIndexer().stop();
//                                        notePlayerSubsystem.shootFromSubwoofer();
                                    }),
                                    notePlayerSubsystem.prepForPickup(),
                                    new InstantCommand(() -> {
                                        swerveDrive.setFaceSpeaker(false);
//                                        notePlayerSubsystem.shootFromSubwoofer();
                                    })
                            );
                            // Keep track of how many times we have tried to get a note from the start
                            grabsFromStartAttempted++;
                        }
                    }
                    if (Objects.equals(autoLane, stageLane)) {
                        addCommands(
                                Commands.runOnce(() -> {
                                    notePlayerSubsystem.rotateArmToDegrees(Constants.NotePlayerConstants.ARM_PICKUP_ANGLE);
                                    notePlayerSubsystem.getShooter().stop();
                                }),
                                Commands.waitSeconds(0.025),
                                Commands.waitUntil(notePlayerSubsystem.getArm()::isAtSetPosition)
                        );
                    }
                    addCommands(
//                        new InstantCommand(() -> DataLogManager.log("Done Scoring! Driving to Center Line!")),
                            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-CL"), pathConstraints)
//                        new InstantCommand(() -> DataLogManager.log("Autonomous Routine is Complete!"))
                    );
                }
            }
        }

        addCommands(Commands.none());

//        double timeTaken = ((System.nanoTime()-startTime)/1E9);

//        DataLogManager.log("****************************************************************************************************************************************************************************************************************************************************************************************\n   Finished Building Auto, Time Taken: " + timeTaken + " Seconds \n*******************************************************************************************************************************");
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

//        // Don't do anything if the number of notes to get is out of bounds
        if(numOfNotesToGet < 1) return  new int[0];
        else if(numOfNotesToGet > totalNumOfNotes) return new int[0];
//
        int[] order = new int[numOfNotesToGet];

//
//        // If we are using the Amp Side Lane, get the notes in ascending order (1-5), note #1 is the one closes to the Amp
//        if(Objects.equals(autoLane, ampLane)) {
//            for(int i = 0; i < numOfNotesToGet; i++) {
//                order[i] = i+1;
//            }
//        }
//
//        // If we are using the Source Side Lane, get the notes in descending order (5-1), note #5 is the one closest to the source
//        else if(Objects.equals(autoLane, sourceLane)) {
//            for(int i = 0; i < numOfNotesToGet; i++) {
//                order[i] = totalNumOfNotes-i;
//            }
//        }
//
//        // If we are using the Under-Stage Lane, start with the middle one, and work our way up, down,
//        // or from the middle outward, depending on the searchDirection
//        else if (Objects.equals(autoLane, stageLane)) {
//            // find the center note
//            int noteNumber = (int) Math.ceil((double) totalNumOfNotes / 2);
//            for(int i = 0; i < numOfNotesToGet; i++) {
//                order[i] = noteNumber;
//
//                if (Objects.equals(searchDirection, "ToAmp")) { // Do we want to go toward the Amp?
//                    noteNumber--;
//                } else if (Objects.equals(searchDirection, "ToSrc")) { // Do we want to go toward the Source?
//                    noteNumber++;
//                } else { // Otherwise go from the middle outward
//                    if (i%2 == 0) {
//                        noteNumber += i+1;
//                    } else {
//                        noteNumber -= i+1;
//                    }
//                }
//
//                // If the note number is out of bounds, reset it to the total number of notes
//                // In the case that we are grabbing all 5 center notes, and the search direction is
//                // -1 (toward the Amp) we would start with 3, then go to 2, and then 1, and the next would be 0,
//                // but since that doesn't exist we would instead roll over to 5, and then go to 4
//                if(noteNumber < 1) noteNumber += totalNumOfNotes;
//                else if(noteNumber > totalNumOfNotes) noteNumber -= totalNumOfNotes;
//            }
//        }

        // Search for notes from the direction of the Amp
        if (Objects.equals(searchDirection, "FromAmp")) {
            for (int i = 0; i < numOfNotesToGet; i++) {
                order[i] = i+1;
            }
        }
        // Search for notes from the direction of the Source
        else if (Objects.equals(searchDirection, "FromSrc")) {
            for (int i = 0; i < numOfNotesToGet; i++) {
                order[i] = totalNumOfNotes-i;
            }
        }
        // Search for notes stating from the center outwards
        else {
            int noteNumber = (int) Math.ceil((double) totalNumOfNotes / 2);
            for (int i = 0; i < numOfNotesToGet; i++) {

                order[i] = noteNumber;

                if (i % 2 == 0) {
                    noteNumber += i + 1;
                } else {
                    noteNumber -= i + 1;
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