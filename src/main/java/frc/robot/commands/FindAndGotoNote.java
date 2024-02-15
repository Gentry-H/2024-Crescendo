package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class FindAndGotoNote extends Command {
    private final NotePlayerSubsystem notePlayerSubsystem;
    private final SwerveDrive swerveDrive;

    private boolean finished = false;

    private PhotonCamera notesCamera;

    public FindAndGotoNote(NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        this.notePlayerSubsystem = notePlayerSubsystem;
        this.swerveDrive = swerveDrive;
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

        long sTime = System.nanoTime();

        // Define rotation needed to center-in on note
        double neededRotation = 0.0d; // in degrees
        double neededSpeed = 0.0d; // 1.0 == 100%
        final double heading = swerveDrive.getHeading().getDegrees();

        System.out.println("... Looking for a note.");

        long eTime = System.nanoTime();
        System.out.println("time for init: " + (eTime -sTime));

        sTime = System.nanoTime();
        PhotonPipelineResult pipelineResult = notesCamera.getLatestResult();
        eTime = System.nanoTime();
        System.out.println("time for getting pipeline: " + (eTime -sTime));

        // Check if limelight has found a target
        if (pipelineResult.hasTargets()) {
            System.out.println("Note found!  ---------------");
            sTime = System.nanoTime();
            PhotonTrackedTarget target = pipelineResult.getBestTarget();
            eTime = System.nanoTime();
            System.out.println("time for getting best target: " + (eTime -sTime));

            sTime = System.nanoTime();
            System.out.println("Pitch: " + target.getPitch());
            System.out.println("Yaw: " + target.getYaw());
            System.out.println("Heading: " + heading);
            eTime = System.nanoTime();
            System.out.println("time for printing pitch, yaw, heading: " + (eTime -sTime));

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

            sTime = System.nanoTime();

            // Select the tollerence (in degrees) of the angle toward the target based on distance (in feet, approximated to pitch)
            int angleTolerance = 2;
            double angleOffset = target.getYaw();

            if(target.getPitch() < -27) {
                System.out.println("Note is at intake.");
                angleTolerance = 1;
                neededSpeed = 0.00;
            }
            else if(target.getPitch() < 0) {
                System.out.println("Note is within a foot");
                angleTolerance = 1;
                neededSpeed = 0.35;
            }
            else if(target.getPitch() < 15) {
                System.out.println("Note is within 3 feet");
                angleTolerance = 1;
                neededSpeed = 0.45;
            }
            else if(target.getPitch() > 15) {
                System.out.println("Note is beyond 3 feet");
                angleTolerance = 2;
                neededSpeed = 0.55;
            }

            neededRotation = angleOffset;

            if(target.getYaw() < -angleTolerance) {
                System.out.println("moving left to target!");
//                neededRotation = angleOffset;
            }
            else if(target.getYaw() > angleTolerance) {
                System.out.println("moving right to target!");
//                neededRotation = -angleOffset;
            }
        }
        else {
            System.out.println("No Note in view...");
            neededSpeed = 0.0;
            neededRotation = 0.0;
        }
        eTime = System.nanoTime();
        System.out.println("time for making decisions: " + (eTime -sTime));

        System.out.println("Desired speed: " + neededSpeed);
        System.out.println("Desired rotation (degrees): " + neededRotation);
        System.out.println("--------------------------------------");

        final double speed = -neededSpeed;
        final double rotation = neededRotation;

        // forward/back (translation), left/right (strafe), slowness, target heading in degrees, robot centric
        swerveDrive.driveSwerveDriveAndRotateToAngle(() -> speed, () -> 0.0 , () -> 0.0, () -> heading + rotation, () -> true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.resetRotationPIDController();
    }
}
