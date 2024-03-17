package frc.robot.subsystems.noteplayer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimator;

import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 *
 */
public class NotePlayerSubsystem extends SubsystemBase implements Constants.NotePlayerConstants, Constants.FieldCoordinates, Constants.GameStateConstants {

    private IndexerModule indexer = new IndexerModule();
    private IntakeModule intake = new IntakeModule();
    private ArmModule arm = new ArmModule();
    private ShooterModule shooter = new ShooterModule();

    private double nextGuessAngle = TRAJECTORY_DEFAULT_INITIAL_ANGLE;

    private double shootingAngle = 90;
    private double shootingSpeed = 1;

    private ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
    private GenericEntry shooterSpeed =
            teleopTab.add("Shooter Speed", 0)
                    .getEntry();

    private GenericEntry shooterAngle =
            teleopTab.add("Shooter Angle", 0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 48))
                    .getEntry();

    private GenericEntry distanceToSpeaker = Shuffleboard.getTab("Teleoperated")
            .add("Distance To Speaker", 2d)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 4))
            .getEntry();

    private final InterpolatingDoubleTreeMap speedInterpolator = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();

    private boolean moveArmInAuto = false;

    public NotePlayerSubsystem() {
        speedInterpolator.put(1.4d, 13d);
        speedInterpolator.put(2d, 14.5d);
        speedInterpolator.put(2.5d, 16d);
        speedInterpolator.put(3d, 17.5d);

        angleInterpolator.put(1.4d, 44d);
        angleInterpolator.put(2d, 35d);
        angleInterpolator.put(2.5d, 30d);
        angleInterpolator.put(3d, 26.5d);

        moveArmInAuto = false;
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
        setRobotStates();
        if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
            if (moveArmInAuto) {
                shooter.spinMetersPerSecond(13);
            } else {
                shootFromSubwoofer();
            }
        }

//        SmartDashboard.putBoolean("LimitSwitchEnabled", indexer.isLimitSwitchEnabled());
//        SmartDashboard.putBoolean("LimitSwitchPressed", indexer.limitSwitchState());

//        SmartDashboard.putNumber("Angle Interpolator", angleInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())));
//        SmartDashboard.putBoolean("Arm At Set Position", arm.isAtSetPosition());
//        SmartDashboard.putBoolean("Shooter At Set Speed", shooter.targetVelocityReached());
//        SmartDashboard.putNumber("Speed Interpolator", speedInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())));
//        SmartDashboard.putNumber("Top Shooter Speed", shooter.getShooterTopVelocityMPS());
//        SmartDashboard.putNumber("Bottom Shooter Speed", shooter.getShooterBottomVelocityMPS());

        SmartDashboard.putBoolean("Indexer Has Note", indexer.noteInIndexer());

        /**
         * PODIUM ANGLE 27.3
         * PODIUM SPEED 17.22
         *
         * SUBWOOFER ANGLE 44
         * SUBWOOFER SPEED 13
         */
    }

    public InterpolatingDoubleTreeMap getAngleInterpolator() {
        return angleInterpolator;
    }

    public InterpolatingDoubleTreeMap getSpeedInterpolator() {
        return speedInterpolator;
    }

    /**
     * Checks whether the target is within range of shooting
     *
     * @param botPose    The {@link Translation2d} of the robot
     * @param targetPose The {@link Translation2d} of the target
     * @return True if the target is within the range of the shooter
     */
    public boolean isWithinRange(Translation2d botPose, Translation2d targetPose) {
        return speedInterpolator.get(botPose.getDistance(targetPose)) <=
                speedInterpolator.get(Double.MAX_VALUE);
    }

    public void setRobotStates() {
//         TODO: Implement the rest of the logic
        switch (RobotState.getInstance().getShooterMode()) {
            case SPEAKER -> {
                RobotState.getInstance().setHasSpeakerTarget(
                        isWithinRange(PoseEstimator.getInstance().getPose().getTranslation(),
                                FieldState.getInstance().onRedAlliance() ? RED_SPEAKER.toTranslation2d() : BLUE_SPEAKER.toTranslation2d()));
                RobotState.getInstance().setShooterStatus(
                        (arm.isAtSetPosition() && shooter.targetVelocityReached() && PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()) < 3) ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
            case AMP -> {
                RobotState.getInstance().setShooterStatus(arm.isAtSetPosition() ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
            case PICKUP -> {
                RobotState.getInstance().setHasNote(intake.noteInIntake() || indexer.noteInIndexer());
                RobotState.getInstance().setShooterStatus(arm.isAtSetPosition() ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
            case EJECT -> {
                RobotState.getInstance().setShooterStatus(
                        (arm.isAtSetPosition() && shooter.targetVelocityReached()) ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
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

    public boolean isMoveArmInAuto() {
        return moveArmInAuto;
    }

    public void setMoveArmInAuto(boolean moveArmInAuto) {
        this.moveArmInAuto = moveArmInAuto;
    }

    // Commands:

    public Command allStop() {
        return this.run(() -> {
            intake.stop();
            indexer.stop();
        });
    }

    public Command intakeNote() {
        return new InstantCommand(() -> indexer.setEnableHardLimit(true)).andThen(new RunCommand(() -> intake.spin(0.5))
                .until(intake::noteInIntake).andThen((
                        new RunCommand(() -> intake.spin(0.3))/*.withTimeout(0.4)
                                .andThen(new RunCommand(() -> intake.spin(0.1)))*/)
                        .alongWith(pullNoteIntoIndexer())))
                .until(indexer::noteInIndexer)//.andThen(new RunCommand(() -> indexer.spin(0.5)).withTimeout(0.2))
                .finallyDo(() -> {
                    intake.stop();
                    indexer.stop();
                    indexer.setEnableHardLimit(false);
                })
                /*)*/.withName("IntakeNote");
    }

    public Command reverseEject() {
        return new InstantCommand(() -> {
            RobotState.getInstance().setShooterMode(ShooterMode.EJECT);
            indexer.setEnableHardLimit(false);
        }).andThen(new RunCommand(
                () -> {
                    indexer.spin(-0.75);
                    intake.spin(-0.5);
                    shooter.spinPercentage(-0.03);
                }
        ).finallyDo(
                () -> {
                    indexer.stop();
                    intake.stop();
                    shooter.stop();
                }
        ));
    }

    public Command forwardEject() {
        return new InstantCommand(() -> {
            RobotState.getInstance().setShooterMode(ShooterMode.EJECT);
            indexer.setEnableHardLimit(false);
        }).andThen(new RunCommand(
                () -> {
                    indexer.spin(1);
                    intake.spin(0.3);
                    shooter.spinMetersPerSecond(4.5);
                }
        ).finallyDo(
                () -> {
                    indexer.stop();
                    intake.stop();
                    shooter.stop();
                }
        ));
    }

    public Command stowShot() {
        return new InstantCommand(() -> {
            arm.rotateToDegrees(15);
            RobotState.getInstance().setShooterMode(ShooterMode.EJECT);
        })
                .andThen(new RunCommand(() -> shooter.spinPercentage(0.7)))
                .until(() -> !indexer.noteInIndexer())
                .unless(() -> !indexer.noteInIndexer())
                .finallyDo(() -> shooter.stop());
    }

    public Command passFromSource() {
        return new InstantCommand(() -> {
            RobotState.getInstance().setShooterMode(ShooterMode.EJECT);
            indexer.setEnableHardLimit(false);
        })
                .andThen(new InstantCommand(() -> arm.rotateToDegrees(35))
                        .alongWith(new RunCommand(() -> shooter.spinMetersPerSecond(11)))
                        .until(() -> RobotState.getInstance().getShooterStatus() == ShooterStatus.READY)
                        .andThen(new RunCommand(() -> shooter.spinMetersPerSecond(20))
                                .alongWith(new RunCommand(() -> indexer.spin(1))))
                        .until(() -> !indexer.noteInIndexer()))
                .finallyDo(() -> {
                    shooter.stop();
                    indexer.stop();
                });
    }

    public void shootFromSubwoofer() {
        shooter.spinMetersPerSecond(26, 4);//22, 4);
    }

    public Command shootFromSubwooferCommand() {
        return new InstantCommand(() -> RobotState.getInstance().setShooterMode(ShooterMode.SPEAKER))
                .andThen(new RunCommand(this::shootFromSubwoofer))
                .finallyDo(() -> shooter.stop());
    }

    public Command pullNoteIntoIndexer() {
        return new RunCommand(
                () -> indexer.spin(1)
        )/*.withTimeout(0.4).andThen(new RunCommand(() -> indexer.spin(0.55)))*/.until(indexer::noteInIndexer);
    }

    public Command feedNoteToShooter() {
        return new RunCommand(
                () -> indexer.spin(1))
                .onlyWhile(indexer::noteInIndexer)
                .andThen(indexer::stop)
                .handleInterrupt(indexer::stop);
    }

    public Command finishShooting() {
        return (new RunCommand(() -> shooter.spinMetersPerSecond(speedInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())))).raceWith(Commands.waitSeconds(0.1))).andThen(() -> shooter.stop());
    }

    public Command prepForPickup() {
        return (new InstantCommand(
                () -> {
                    RobotState.getInstance().setShooterMode(ShooterMode.PICKUP);
                    indexer.setEnableHardLimit(true);
                })
                .alongWith(rotateArmToDegrees(ARM_PICKUP_ANGLE)));
    }

    public Command spinUpShooterForSpeaker() {
        return new RunCommand(
                () -> shooter.spinMetersPerSecond(speedInterpolator.get(
                        Math.abs(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()))))
        ).finallyDo(() -> shooter.stop());
    }

    public Command aimAndSpinUpForSpeaker() {
        return (new RunCommand(
                () -> {
                    RobotState.getInstance().setShooterMode(ShooterMode.SPEAKER);
                    indexer.setEnableHardLimit(false);
                    shooter.spinMetersPerSecond(speedInterpolator.get(Math.abs(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()))));
                    arm.rotateToDegrees(angleInterpolator.get(Math.abs(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()))));
//                    shooter.spinMetersPerSecond(speedInterpolator.get(distanceToSpeaker.getDouble(1.4)));
//                    arm.rotateToDegrees(angleInterpolator.get(distanceToSpeaker.getDouble(1.4)));

//                    shooter.spinMetersPerSecond(shooterSpeed.getDouble(0));
//                    arm.rotateToDegrees(shooterAngle.getDouble(0));
                }
        )).finallyDo(() -> {
                            shooter.stop();
                        }
                ).onlyIf(indexer::noteInIndexer).onlyWhile(indexer::noteInIndexer)
                .withName("AimAndSpinUpForSpeaker");
    }

    public Command scoreNote() {
        return new InstantCommand(() -> {
            switch (RobotState.getInstance().getShooterMode()) {
                case SPEAKER -> {
                    System.out.println("Scoring Speaker");
                    CommandScheduler.getInstance().schedule(feedNoteToShooter().andThen(finishShooting()));
                }
                case AMP -> {
                    System.out.println("Scoring Amp");
                    CommandScheduler.getInstance().schedule(scoreAmp());
                }
                default -> {
                    System.out.println("Scoring Nothing");
                }
            }
        });
    }

    public Command prepForAmp() {
        return rotateArmToDegrees(ARM_AMP_ANGLE)
                .alongWith(new InstantCommand(
                        () -> {
                            RobotState.getInstance().setShooterMode(ShooterMode.AMP);
                            indexer.setEnableHardLimit(false);
                        }))
                .onlyIf(indexer::noteInIndexer)
                .onlyWhile(indexer::noteInIndexer)
                .withName("PrepForAmp");
    }

    public Command scoreAmp() {
        return new RunCommand(
                () -> {
                    shooter.spinPercentage(0.2);
                    indexer.spin(1);
                }
        ).onlyWhile(indexer::noteInIndexer).onlyIf(indexer::noteInIndexer)
                .finallyDo(() -> {
                    shooter.stop();
                    indexer.stop();
                });
    }

    public Command rotateArmToDegrees(double degrees) {
        return new InstantCommand(
                () -> arm.rotateToDegrees(degrees)
        );
    }

    public Command autoPrepArmForShooting() {
        return new InstantCommand(
                () -> arm.rotateToDegrees(angleInterpolator.get(1.4))
        );
    }

    public Command driveArmPercent(DoubleSupplier percent) {
        return (new RunCommand(
                () -> arm.rotatePercentOut(percent.getAsDouble()))
                .finallyDo(() -> arm.rotatePercentOut(0)));
    }
}
