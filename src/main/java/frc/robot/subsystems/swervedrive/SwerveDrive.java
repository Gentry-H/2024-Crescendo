package frc.robot.subsystems.swervedrive;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.FieldState;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PoseEstimator;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Swerve Drive Subsystem -- based on team 364's code
 */
public class SwerveDrive extends SubsystemBase implements Constants.Swerve, Constants.PathPlannerConstants, Constants.DriverControls {
    public SwerveModule[] swerveMods;
    private final AHRS gyro;

    private final PoseEstimator poseEstimator;

    private final Field2d field = new Field2d();

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

    private final PIDController rotationPIDController;

    private int invertJoystickInputs;

    private Rotation2d holdHeading;
    private boolean faceSpeaker = false;
    private boolean shouldUseVision = true;

    private int controlsInvert;

    public SwerveDrive() {
        gyro = NavX.getNavX();
        gyro.reset();

        swerveMods = new SwerveModule[]{
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };

        resetModulesToAbsolute();

        poseEstimator = PoseEstimator.getInstance();

        rotationPIDController = new PIDController(0.04d, 0.00001d, 0.004d);
        rotationPIDController.setIZone(1);
        rotationPIDController.enableContinuousInput(-180, 180);
        rotationPIDController.setTolerance(0.1);

        // Configure PathPlanner Auto Builder
        AutoBuilder.configureHolonomic(
                poseEstimator::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                pathFollowerConfig,
                () -> FieldState.getInstance().onRedAlliance(),
                this
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

//        SmartDashboard.putData("Field", field);

        if (FieldState.getInstance().onRedAlliance()) {
            invertJoystickInputs = 1;
        } else {
            invertJoystickInputs = -1;
        }

        controlsInvert = 1;

        holdHeading = getHeading();

        shouldUseVision = true;
    }

    /**
     * Drives the {@link SwerveDrive} Subsystem
     *
     * @param translation   A {@link Translation2d} representing the desired X and Y speed in meters per second
     * @param rotation      The desired angular velocity in radians per second
     * @param fieldRelative Controls whether it should be driven in field or robot relative mode
     * @param isOpenLoop    Controls whether it drives in open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                swerveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getHeading()
                        )
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Sets the states of each module
     *
     * @param desiredStates An array of the {@link SwerveModuleState} for each {@link SwerveModule}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Turns all the wheels to face inwards creating and "X" shape to lock the robot in place if the robot is not currently moving
     */
    public void xLockWheels() {
        if (getModuleStates()[0].speedMetersPerSecond < 0.1) {
            SwerveModuleState[] desiredStates = {
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
            };
            for (SwerveModule mod : swerveMods) {
                mod.setAngle(desiredStates[mod.moduleNumber]);
            }
        }
    }

    /**
     * @return An array containing the {@link SwerveModuleState} of each {@link SwerveModule}
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return An array of the {@link SwerveModulePosition} of each {@link SwerveModule}
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * @param pose The {@link Pose2d} to set the {@link SwerveDrivePoseEstimator} to
     */
    public void setPose(Pose2d pose) {
        poseEstimator.setPose(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * @return The speeds of each {@link SwerveModule} as {@link ChassisSpeeds} Required by Path Planner
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Required by Path Planner, drives the robot given {@link ChassisSpeeds}
     *
     * @param speeds the ChassisSpeeds to drive the robot with
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        setModuleStates(states);
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }

    /**
     * @return The heading reported by the {@link SwerveDrivePoseEstimator} as a {@link Rotation2d}
     */
    public Rotation2d getHeading() {
        return poseEstimator.getPose().getRotation();
    }

    /**
     * Sets the {@link SwerveDrivePoseEstimator} heading
     *
     * @param heading The heading of the robot
     */
    public void setHeading(Rotation2d heading) {
        poseEstimator.setPose(getGyroYaw(), getModulePositions(), new Pose2d(poseEstimator.getPose().getTranslation(), heading));
    }

    /**
     * Resets the {@link SwerveDrivePoseEstimator} heading
     */

    public void zeroHeading(){
        poseEstimator.setPose(getGyroYaw(), getModulePositions(), new Pose2d(poseEstimator.getPose().getTranslation(), new Rotation2d()));
    }

    public void setHoldHeading(Rotation2d holdHeading) {
        this.holdHeading = holdHeading;
    }

    public Rotation2d getHoldHeading() {
        return holdHeading;
    }

    public void setFaceSpeaker(boolean faceSpeaker) {
        this.faceSpeaker = faceSpeaker;
    }

    public boolean isFacingSpeaker() {
        return faceSpeaker && rotationPIDController.atSetpoint();
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        if (faceSpeaker) {
            return Optional.of(getAngleToPose(FieldState.getInstance().getSpeakerCoords().toTranslation2d()));
        } else {
            return Optional.empty();
        }
    }

    /**
     * @return The Robot yaw as reported by the NavX
     */
    public Rotation2d getGyroYaw() {
        return invertGyro ? (Rotation2d.fromDegrees(-gyro.getYaw())) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * @return The angular velocity of the robot as reported by the NavX
     */
    public float getGyroYawSpeed() {
        return invertGyro ? -gyro.getRawGyroZ() : gyro.getRawGyroZ();
    }

    /**
     * Resets each {@link SwerveModule} based on the {@link CANcoder} and angle offset
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    /**
     * Used for driving the robot during teleop
     *
     * @param translationVal  A double for the forwards/backwards speed as a percentage
     * @param strafeVal       A double for the left/right speed as a percentage
     * @param slowVal         A double for the amount to slow the robot by
     * @param manualRotationVal     A double for the angular velocity as a percentage
     * @param robotCentricSup A double for driving in robot or field centric mode
     */
    public void teleopDriveSwerveDrive(double translationVal, double strafeVal, double slowVal,
                                       double manualRotationVal, boolean robotCentricSup) {
        translationVal *= (1 - (slowVal * 0.75));
        strafeVal *= (1 - slowVal * 0.75);
        double rotationVal;
        if (faceSpeaker) {
            Rotation2d offsetAngleToSpeaker = getAngleToPose(FieldState.getInstance().getSpeakerCoords().toTranslation2d());
            if (FieldState.getInstance().onRedAlliance()) {
                if (offsetAngleToSpeaker.getRadians() < 0) offsetAngleToSpeaker = Rotation2d.fromDegrees(180).plus(offsetAngleToSpeaker);

                else offsetAngleToSpeaker = Rotation2d.fromDegrees(180).minus(offsetAngleToSpeaker);
            }

            // TODO: find the actual angle
            offsetAngleToSpeaker = offsetAngleToSpeaker.div((50d/2d)).plus(Rotation2d.fromDegrees(4.5));

            SmartDashboard.putNumber("Offset Angle", offsetAngleToSpeaker.getDegrees());

            holdHeading = getAngleToPose(FieldState.getInstance().getSpeakerCoords().toTranslation2d());//.plus(offsetAngleToSpeaker);
        }
        if (manualRotationVal != 0) {
            rotationVal = manualRotationVal;
            holdHeading = getHeading();
        }
        else if (Math.abs(translationVal) > 0.02 || Math.abs(strafeVal) > 0.02) {
            rotationVal = rotationPercentageFromTargetAngle(holdHeading);
        } else {
            rotationVal = manualRotationVal;
        }

        drive(new Translation2d(translationVal, strafeVal).times(maxSpeed),
                rotationVal * maxAngularVelocity,
                !robotCentricSup,
                false);
    }

    /**
     * Creates an angular velocity based on a target angle, and the direction the robot is currently facing
     *
     * @param targetAngle The angle the robot should turn to
     * @return The angular velocity as a percentage
     */
    public double rotationPercentageFromTargetAngle(Rotation2d targetAngle) {
        return MathUtil.clamp(rotationPIDController.calculate(getHeading().getDegrees(),
                targetAngle.getDegrees() % 360) * 0.1, -0.1, 0.1);
    }

    public void resetRotationPIDController() {
        rotationPIDController.reset();
    }

    /**
     * Get the angle to a given position
     *
     * @param coords The coordinates to return the angle to
     * @return The angle from your current position to the given coordinates
     */
    public Rotation2d getAngleToPose(Translation2d coords) {
        return poseEstimator.getPose().getTranslation().minus(coords).getAngle();
    }

    private DoubleSupplier speedsFromJoysticks(DoubleSupplier rawSpeedSup, BooleanSupplier robotCentric) {
        return () -> (((FieldState.getInstance().onRedAlliance()&&!robotCentric.getAsBoolean()) ? 1 : -1) * controlsInvert
                * Math.copySign(Math.pow(Math.abs(MathUtil.applyDeadband(rawSpeedSup.getAsDouble(), stickDeadband)),
                swerveSensitivityExponent), rawSpeedSup.getAsDouble()) * swerveSpeedMultiplier);
    }

    public void setShouldUseVision(boolean shouldUseVision) {
        this.shouldUseVision = shouldUseVision;
    }

    public boolean isShouldUseVision() {
        return shouldUseVision;
    }

    // **** Commands ****

    /**
     * Used for driving the robot during teleop while rotating to the angle reported by the D-Pad of the controller
     *
     * @param rawTranslationSup {@link DoubleSupplier} for the forwards/backwards speed as a percentage
     * @param rawStrafeSup      {@link DoubleSupplier} for the left/right speed as a percentage
     * @param rawRotationSup    {@link DoubleSupplier} for rotation speed as a percentage
     * @param robotCentricSup   {@link BooleanSupplier} for driving in robot or field centric mode
     */
    public Command teleopDriveSwerveDriveCommand(DoubleSupplier rawTranslationSup, DoubleSupplier rawStrafeSup, DoubleSupplier rawSlowSup,
                                                 DoubleSupplier rawRotationSup, BooleanSupplier robotCentricSup) {
        DoubleSupplier rotationSup = () -> (-1 *
                Math.copySign(Math.pow(Math.abs(MathUtil.applyDeadband(rawRotationSup.getAsDouble(), stickDeadband))
                        , swerveSensitivityExponent), rawRotationSup.getAsDouble()) * swerveRotationMultiplier);

        DoubleSupplier slowSup = () -> MathUtil.applyDeadband(rawSlowSup.getAsDouble(), stickDeadband);

        return this.run(() -> teleopDriveSwerveDrive(
                speedsFromJoysticks(rawTranslationSup, robotCentricSup).getAsDouble(),
                speedsFromJoysticks(rawStrafeSup, robotCentricSup).getAsDouble(),
                slowSup.getAsDouble(),
                rotationSup.getAsDouble(),
                robotCentricSup.getAsBoolean()
        ));
    }

    /**
     * Used for rotating to the angle reported by the D-Pad of the controller
     */
    public Command setHoldHeading(double holdHeading) {
        return new InstantCommand(() -> setHoldHeading(Rotation2d.fromDegrees(holdHeading)));
    }

    public void driveSwerveDriveAndRotateToAngle(double translation, double strafe, double targetDegrees) {
        drive(new Translation2d(translation, strafe).times(maxSpeed),
                -rotationPercentageFromTargetAngle(Rotation2d.fromDegrees(targetDegrees))*maxAngularVelocity,
                false,
                false);
    }

    public Command alignWithAmp() {
        AtomicReference<PathPlannerPath> pathToAmp = new AtomicReference<>(PathPlannerPath.fromPathFile("AlignAmpX"));

        return new InstantCommand(() -> {
            if (FieldState.getInstance().onRedAlliance()) {
                if (PoseEstimator.getInstance().getPose().getX() > Units.inchesToMeters(578.77-24)) {
                    pathToAmp.set(PathPlannerPath.fromPathFile("AlignAmpY"));
                } else {
                    pathToAmp.set(PathPlannerPath.fromPathFile("AlignAmpX"));
                }
            } else {
                if (PoseEstimator.getInstance().getPose().getX() < Units.inchesToMeters(72.5+24)) {
                    pathToAmp.set(PathPlannerPath.fromPathFile("AlignAmpY"));
                } else {
                    pathToAmp.set(PathPlannerPath.fromPathFile("AlignAmpX"));
                }
            }
        }).andThen(AutoBuilder.followPath(pathToAmp.get())).withName("AlignWithAmp");
    }

    /**
     * Enables facing the speaker when called, and disables when interrupted
     */
    public Command faceSpeaker() {
        return Commands.startEnd(() -> faceSpeaker=true, () -> faceSpeaker=false).withName("faceSpeaker");
    }

    public Command invertControls() {
        return new InstantCommand(() -> controlsInvert *= -1);
    }

    @Override
    public void periodic() {
        poseEstimator.updateSwerveEstimator(getGyroYaw(), getModulePositions());

        // Correct pose estimate with vision measurements
        if (shouldUseVision) {
            poseEstimator.updateWithVision();
        }

        posePublisher.set(poseEstimator.getPose());

        SmartDashboard.putNumber("Distance To Speaker", Math.abs(poseEstimator.getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())));
        SmartDashboard.putNumber("Hold Heading", holdHeading.getDegrees());
        SmartDashboard.putNumber("Current Heading", getHeading().getDegrees());

        if (RobotState.isDisabled()) {
            setHoldHeading(getHeading());

            invertJoystickInputs = FieldState.getInstance().onRedAlliance() ? 1 : -1;
            SmartDashboard.putNumber("Rotation", getHeading().getDegrees());
        }

//        SmartDashboard.putNumber("Swerve Estimator X", poseEstimator.getPose().getX());
//        SmartDashboard.putNumber("Swerve Estimator Y", poseEstimator.getPose().getY());
//
//        System.out.println("Swerve Pose: " + poseEstimator.getPose());
/*
        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", Math.abs(mod.getState().speedMetersPerSecond));
        }
*/
    }
}