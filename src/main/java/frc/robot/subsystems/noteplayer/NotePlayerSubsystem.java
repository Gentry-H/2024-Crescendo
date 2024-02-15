package frc.robot.subsystems.noteplayer;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.PoseEstimator;

import java.awt.geom.Line2D;

import static edu.wpi.first.units.Units.*;

/**
 *
 */
public class NotePlayerSubsystem extends SubsystemBase implements Constants.NotePlayerConstants, Constants.FieldCoordinates, Constants.GameStateConstants {

    private IndexerModule indexer = new IndexerModule();
    private IntakeModule intake = new IntakeModule();
    private ArmModule arm = new ArmModule();
    private ShooterModule shooter = new ShooterModule();

//    SysIdRoutine sysIdRoutine = new SysIdRoutine(
//            new SysIdRoutine.Config(Volts.of(0.4).per(Second.of(1)), Volts.of(5), Second.of(10)),
//            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> arm.driveVolts(volts),
//                    log -> {
//                        log.motor("leftArmMotor")
//                                .voltage(
//                                        arm.getLeftMotorVolts()
//                                )
//                                .angularPosition(arm.getPositionRadians())
//                                .angularVelocity(arm.getSpeedRadians().per(Second));
//                        log.motor("rightArmMotor")
//                                .voltage(
//                                        arm.getRightMotorVolts()
//                                )
//                                .angularPosition(arm.getPositionRadians())
//                                .angularVelocity(arm.getSpeedRadians().per(Second));
//                    }, this)
//    );

    public NotePlayerSubsystem() {
    }

    public IntakeModule getIntake() {
        return intake;
    }

    public ShooterModule getShooter() {
        return shooter;
    }

    public IndexerModule getIndexer() {
        return indexer;
    }

    public ArmModule getArm() {
        return arm;
    }

    @Override
    public void periodic() {
        arm.periodic();
    }

    /**
     * Calculate the position of the Shooter Wheels relative to the target
     *
     * @param robotDistance The horizontal distance from the center of the robot to the target
     * @param angle         The angle of the Arm as a {@link Rotation2d}
     * @return The position of the shooter as a {@link Translation2d}, X for distance, Y for height above floor
     */
    public Translation2d calculateShooterTargetRelativePosition(double robotDistance, Rotation2d angle) {
        double verticalOffset = (-angle.getSin() * SHOOTER_ARM_LENGTH) +
                (angle.getCos() * SHOOTER_ARM_TO_WHEELS_LENGTH) +
                SHOOTER_VERTICAL_OFFSET;

        double horizontalOffset = (angle.getCos() * SHOOTER_ARM_LENGTH) +
                (angle.getSin() * SHOOTER_ARM_TO_WHEELS_LENGTH) +
                SHOOTER_HORIZONTAL_OFFSET;

        return new Translation2d(robotDistance + horizontalOffset, verticalOffset);
    }

    /**
     * Calculates the optimal shooting angle and speed for the apex of the projectile to be at the given coordinates
     * - All distances and speeds in meters
     *
     * @param robotPose    The current {@link Pose2d} of the robot in meters
     * @param targetCoords The coordinates of the target as a {@link Translation3d} in meters
     * @return The target speed and angle as a {@link Translation2d} (use the getNorm method for the speed, and the getAngle method for the angle)
     */
    public Translation2d calculateShootingParameters(Pose2d robotPose, Translation3d targetCoords) { // The brute force calculations

        // Get the horizontal distance from the robot to the target
        double robotToTargetDistance = robotPose.getTranslation().getDistance(targetCoords.toTranslation2d());

        // The shooter angle, We tested every angle incremented by 0.1 between 1 and 66, 21.7 gave
        // the lowest average number of loops before finding the optimal parameters
        Rotation2d shooterAngle = Rotation2d.fromDegrees(25);

        // Pose2d representing the position of the shooter.
        // X for distance to target, Y for height above ground, Rotation2d for the angle
        Pose2d shooterPose = new Pose2d(
                calculateShooterTargetRelativePosition(robotToTargetDistance, shooterAngle),
                shooterAngle);

        // Calculate the launch speed in meters per second needed for the apex to be at the target height
        // given the height of the target and the shooter, and the angle of the shooter
        double launchSpeed = Math.sqrt(
                (targetCoords.getZ() - shooterPose.getY()) * 9.8 * 2 / Math.pow(shooterAngle.getSin(), 2));

        // Calculate the time in seconds from launching the projectile to it reaching the apex
        double timeToApex = launchSpeed * shooterAngle.getSin() / 9.8;

        // Calculate the horizontal distance in meters to the apex
        double launchDistance = launchSpeed * shooterAngle.getCos() * timeToApex;

        // Calculate the error of the expected launch distance from the target  launch distance
        double distanceError = shooterPose.getX() - launchDistance;

        // Do all of that over and over again until the distance error is less than a centimeter
        while (distanceError > 0.01 || distanceError < -0.01) {
            // Adjust the shooter angle according to how for off the distance is.
            // The distance error must be divided by at least 2, any less than that and the
            // angle will end up oscillating too much, the while loop will take to long, and the code will crash.
            // Tested dividing by every number between 10 and 1, 2.61 was optimal with a starting angle of 21.7
            shooterAngle = Rotation2d.fromRadians(shooterAngle.getRadians() * 1 - (distanceError / shooterPose.getX()) / 2.61);

            // Catch the angle going out of bounds.
            if (shooterAngle.getDegrees() >= 90) {
                // If the angle is >= 90 the projectile will have no positive horizontal speed
                System.out.println("*******************SHOOTER ANGLE OUT OF BOUNDS!!!!! *********************");
                shooterAngle = Rotation2d.fromDegrees(85);
            } else if (shooterAngle.getDegrees() <= 0) {
                // If the angle is <= 0 the projectile will have no positive vertical speed
                System.out.println("*******************SHOOTER ANGLE OUT OF BOUNDS!!!!! *********************");
                shooterAngle = Rotation2d.fromDegrees(5);
            }
            shooterPose = new Pose2d(
                    calculateShooterTargetRelativePosition(robotToTargetDistance, shooterAngle),
                    shooterAngle);

            launchSpeed = Math.sqrt(
                    (targetCoords.getZ() - shooterPose.getY()) * 9.8 * 2 / Math.pow(shooterAngle.getSin(), 2));

            timeToApex = launchSpeed * shooterAngle.getSin() / 9.8;

            launchDistance = launchSpeed * shooterAngle.getCos() * timeToApex;

            distanceError = shooterPose.getX() - launchDistance;
        }

        /*
            When finding the optimal starting angle and number to divide the distance error by, we started
            with a 1-degree angle and 2 for the divisor. We then ran tests in batches of 300, with random
            robot poses (X of 4-44, Y of 5-26), and calculated the average number of times we had to calculate the shooting parameters
            including the initial calculations before the while loop. We automatically tested every combination
            of angle and divisor (0.1 increment for the angle, 0.01 increment for the divisor), and found the
            combination that resulted in the lowest average number of loops, and the lowest maximum number of
            loops. This gave us and angle of 21.7 and a divisor of 2.61, which had a maximum number of calculations
            of 6, and an average of 3.97
         */

        // Return the shooting parameters as a vector in Translation2d form
        return new Translation2d(launchSpeed, shooterAngle);
    }

    /**
     * Checks whether the stage is between the robot and the target
     *
     * @param botPose    The {@link Translation2d} of the robot
     * @param targetPose The {@link Translation2d} of the target
     * @return True if the stage is not in the way
     */
    public boolean hasLineOfSight(Translation2d botPose, Translation2d targetPose) {
        Rotation2d angleToTarget = botPose.minus(targetPose).getAngle();
        Translation2d perpendicularOffset;
        if (botPose.getY() > RED_STAGE_SPEAKER_POINT.getY()) {
            perpendicularOffset = new Translation2d(GAME_PIECE_NOTE_DIAMETER / 2, angleToTarget.minus(Rotation2d.fromDegrees(90)));
        } else {
            perpendicularOffset = new Translation2d(GAME_PIECE_NOTE_DIAMETER / 2, angleToTarget.plus(Rotation2d.fromDegrees(90)));
        }

        Line2D notePath = new Line2D.Double(botPose.getX() + perpendicularOffset.getX(), botPose.getY() + perpendicularOffset.getY(),
                targetPose.getX() + perpendicularOffset.getX(), targetPose.getY() + perpendicularOffset.getY());

        if (FieldState.getInstance().onRedAlliance()) {
            return !(notePath.intersectsLine(RED_STAGE_RIGHT) || notePath.intersectsLine(RED_CENTER_STAGE) || notePath.intersectsLine(RED_STAGE_LEFT));
        } else {
            return !(notePath.intersectsLine(BLUE_STAGE_RIGHT) || notePath.intersectsLine(BLUE_CENTER_STAGE) || notePath.intersectsLine(BLUE_STAGE_LEFT));
        }
    }

    public void setRobotStates() {
        // TODO: Implement the rest of the logic
        switch (RobotState.getInstance().getShooterMode()) {
            case SPEAKER:
                RobotState.getInstance().setHasSpeakerTarget(
                        hasLineOfSight(PoseEstimator.getInstance().getPose().getTranslation(),
                                FieldState.getInstance().onRedAlliance() ? RED_SPEAKER.toTranslation2d() : BLUE_SPEAKER.toTranslation2d()));
                RobotState.getInstance().setShooterStatus(
                        (arm.isAtSetPosition() && shooter.targetVelocityReached()) ? ShooterStatus.READY : ShooterStatus.UNREADY);
        }

        RobotState.getInstance().setHasNote(intake.noteInIntake() || indexer.noteInIndexer());
    }

    /**
     * This is what we used for generating random robot poses for tuning the
     * calculateShootingParameters method
     *
     * @return A {@link Pose2d} with random coordinates on the blue alliance side of the field
     */
    public Pose2d generateRandomBotPose() {
        return new Pose2d(
                Units.feetToMeters(Math.random() * 23 + 4),
                Units.feetToMeters(Math.random() * 21 + 5),
                new Rotation2d()
        );
    }

    // Commands:

    public Command allStop() {
        return this.run(() -> {
            intake.stopSpinning();
            indexer.stop();
        });
    }

    public Command intakeNote() {
        return ((new RunCommand(() -> intake.spin(0.3))
                .until(intake::noteInIntake).andThen(
                        new RunCommand(() -> intake.spin(0.15))
                                .raceWith(pullNoteIntoIndexer())))
        );
    }

    public Command pullNoteIntoIndexer() {
        return new RunCommand(
                () -> indexer.spin(1)
        ).until(indexer::noteInIndexer);
    }

    public Command feedNoteToShooter() {
        return new RunCommand(
                () -> indexer.spin(1))
                .onlyWhile(indexer::noteInIndexer);
    }

    public Command spinUpShooter() {
        return ((new RunCommand(
                () -> shooter.spinPercentage(0.75)
        ).raceWith(Commands.waitSeconds(0.50)).onlyIf(indexer::noteInIndexer)));
//                .andThen(() -> shooter.shoot(0.50)).raceWith(Commands.waitSeconds(0.25))).andThen(() -> shooter.stop());
    }

    public Command takeShot() {
        return ((new RunCommand(
                () -> shooter.spinPercentage(0.75)
        ).onlyWhile(indexer::noteInIndexer))
                .andThen(() -> shooter.spinPercentage(0.75)).raceWith(Commands.waitSeconds(0.25))).andThen(() -> shooter.stop());
    }

    public Command aimAtTarget() {
        Translation3d targetCoords = new Translation3d(
                Units.inchesToMeters(-1.5),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(80)
        );

        return new InstantCommand(() -> this.calculateShootingParameters(generateRandomBotPose(), targetCoords));
    }

    public Command rotateArmToDegrees(double degrees) {
        return new RunCommand(
                () -> arm.rotateToDegrees(degrees)
        );
    }

//    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction) {
//        return sysIdRoutine.quasistatic(direction);
//    }
//
//    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//        return sysIdRoutine.dynamic(direction);
//    }
}
