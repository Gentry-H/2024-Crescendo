package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;

public class AutoScoreInSpeaker extends SequentialCommandGroup {

    public AutoScoreInSpeaker(NotePlayerSubsystem notePlayerSubsystem) {
        addCommands(
                new InstantCommand(() -> RobotState.getInstance().setShooterMode(Constants.GameStateConstants.ShooterMode.SPEAKER)),
//                new AutoLog("Starting Scoring!"),
                Commands.run(() -> {
                    double distanceToSpeaker = PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d());
                    notePlayerSubsystem.getShooter().spinMetersPerSecond(notePlayerSubsystem.getSpeedInterpolator().get(distanceToSpeaker));
                    notePlayerSubsystem.getArm().rotateToDegrees(notePlayerSubsystem.getAngleInterpolator().get(distanceToSpeaker));
//                    notePlayerSubsystem.shootFromSubwoofer();
                        })
                        .alongWith((Commands.waitSeconds(0.06)
                                .andThen(Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY)))
                                .andThen(/*new AutoLog("Scoring!"),*/ notePlayerSubsystem.scoreNote()))
                        .raceWith(Commands.waitUntil(() -> !RobotState.getInstance().hasNote())
                                /*.andThen(Commands.waitSeconds(0.1), new AutoLog("Finished Scoring!"))*/)
                        .finallyDo(notePlayerSubsystem::shootFromSubwoofer));
    }
}
