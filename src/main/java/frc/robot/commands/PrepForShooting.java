package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bluecrew.util.FieldState;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class PrepForShooting extends Command {

    private final SwerveDrive swerveDrive;
    private final NotePlayerSubsystem notePlayerSubsystem;
    private Translation3d adjustedSpeakerCoords;

    private final InterpolatingDoubleTreeMap shooterAngleInterpolator;
    private final InterpolatingDoubleTreeMap shooterSpeedInterpolator;

    public PrepForShooting(SwerveDrive swerveDrive, NotePlayerSubsystem notePlayerSubsystem) {
        this.swerveDrive = swerveDrive;
        this.notePlayerSubsystem = notePlayerSubsystem;

        adjustedSpeakerCoords = FieldState.getInstance().getSpeakerCoords();

        shooterSpeedInterpolator = notePlayerSubsystem.getSpeedInterpolator();
        shooterAngleInterpolator = notePlayerSubsystem.getAngleInterpolator();
    }

    @Override
    public void initialize() {
        swerveDrive.setFaceSpeaker(true);
    }

    @Override
    public void execute() {
        ChassisSpeeds swerveSpeeds = swerveDrive.getFieldRelativeSpeeds();
        double distanceToTarget = PoseEstimator.getInstance().getPose().getTranslation().getDistance(adjustedSpeakerCoords.toTranslation2d());

        for (int i = 0; i < 10; i++) {
            distanceToTarget = PoseEstimator.getInstance().getPose().getTranslation().getDistance(adjustedSpeakerCoords.toTranslation2d());

            double time = distanceToTarget / shooterSpeedInterpolator.get(distanceToTarget);

            Translation2d positionChange =
                    new Translation2d(swerveSpeeds.vxMetersPerSecond * time,
                            swerveSpeeds.vyMetersPerSecond * time);

            adjustedSpeakerCoords =
                    FieldState.getInstance().getSpeakerCoords().minus(
                            new Translation3d(positionChange.getX(), positionChange.getY(), 0)
                    );
        }

        notePlayerSubsystem.getArm().rotateToDegrees(shooterAngleInterpolator.get(distanceToTarget));
        notePlayerSubsystem.getShooter().spinMetersPerSecond(shooterSpeedInterpolator.get(distanceToTarget));

        swerveDrive.setSpeakerCoords(adjustedSpeakerCoords);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setFaceSpeaker(false);
        swerveDrive.setSpeakerCoords(FieldState.getInstance().getSpeakerCoords());
        notePlayerSubsystem.getShooter().stop();
    }
}
