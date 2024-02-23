package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class FindAndGotoNote extends Command {
    private final NotePlayerSubsystem notePlayerSubsystem;
    private final SwerveDrive swerveDrive;

    private boolean alsoPickupNote;

    private boolean finished = false;

    private PhotonCamera notesCamera;

    public FindAndGotoNote(NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive /*, boolean alsoPickupNote*/) {
        this.notePlayerSubsystem = notePlayerSubsystem;
        this.swerveDrive = swerveDrive;
        this.alsoPickupNote = alsoPickupNote;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.notePlayerSubsystem, this.swerveDrive);

        notesCamera = VisionModule.getInstance().getNotesIndexerCamera();
    }

    Command defDriveCommand;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Define rotation needed to center-in on note
        double neededRotation = 0.0d; // in degrees
        double neededSpeed = 0.0d; // 1.0 == 100%
        final double heading = swerveDrive.getHeading().getDegrees();

//        System.out.println("... Looking for a note.");

        PhotonPipelineResult pipelineResult = notesCamera.getLatestResult();

        // Check if limelight has found a target
        if (pipelineResult.hasTargets()) {
            // blink the blinkin
            RobotState.getInstance().setNoteIsAvailable(true);

//            System.out.println("Note found!  ---------------");
            PhotonTrackedTarget target = pipelineResult.getBestTarget();

//            System.out.println("Pitch: " + target.getPitch());
//            System.out.println("Yaw: " + target.getYaw());
//            System.out.println("Heading: " + heading);

            /*
                Observed pitches and yaws from the indexer camera with a note place at the following distances

                Pitches
                =======
                0 ft in front = -23.5 pitch
                1 ft in front = -0.90
                2 ft in front = 9.5
                3 ft in front = 15.0
                4 ft in front = 18.4
                5 ft in front = 21.3

                Yaws
                ======
                0 ft to right = 0 yaw (at 18" distance away from front)
                1 ft to right = 25.0
                2 ft to right = 43.25
                2 ft to right = 24.9 (at 36" distance away from front)

             */

            double angleOffset = target.getYaw();

            if(target.getPitch() < -27) {
//                System.out.println("Note is at intake.");
                neededSpeed = 0.00;
            }
            else if(target.getPitch() < 0) {
//                System.out.println("Note is within a foot");
                neededSpeed = 0.19;
            }
            else if(target.getPitch() < 15) {
//                System.out.println("Note is within 3 feet");
                neededSpeed = 0.3;
            }
            else if(target.getPitch() > 15) {
//                System.out.println("Note is beyond 3 feet");
                neededSpeed = 0.44;
            }

            neededRotation = angleOffset;

/*
            if(target.getYaw() < -angleTolerance) {
                System.out.println("moving left to target!");
//                neededRotation = angleOffset;
            }
            else if(target.getYaw() > angleTolerance) {
                System.out.println("moving right to target!");
//                neededRotation = -angleOffset;
            }
 */
        }
        else {
            System.out.println("No Note in view...");
            neededSpeed = 0.0;
            neededRotation = 0.0;
            // blink the blinkin
            RobotState.getInstance().setNoteIsAvailable(false);
        }

//        System.out.println("Desired speed: " + neededSpeed);
//        System.out.println("Desired rotation (degrees): " + neededRotation);
//        System.out.println("--------------------------------------");

        final double speed = -neededSpeed;
        final double rotation = neededRotation;

        // forward/back (translation), left/right (strafe), target heading in degrees
        swerveDrive.driveSwerveDriveAndRotateToAngle(speed, 0.0, heading + rotation);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.resetRotationPIDController();
    }
}
