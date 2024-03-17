package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 *
 */
public class ClimberSubsystem extends SubsystemBase implements Constants.ElevatorConstants {

    private final TalonFX motor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    private final TalonFX motor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);

    private VelocityVoltage climberVelocity = new VelocityVoltage(0);

    private SimpleMotorFeedforward climberFeedForward = new SimpleMotorFeedforward(elevatorKS, elevatorKV, elevatorKA);

    private PositionVoltage climberPositionVoltage = new PositionVoltage(0);

    private final DutyCycleOut climberDutyCycle = new DutyCycleOut(0);

    private static final ClimberSubsystem climberInstance = new ClimberSubsystem();

    private TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    private final Servo elevatorStopper;

    private boolean climbingChain;

    private final AHRS navX;

    private ClimberSubsystem() {
        motor1.clearStickyFaults();
        motor2.clearStickyFaults();

        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
        climberConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ELEVATOR_MOTOR_UPPER_LIMIT_POS;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ELEVATOR_MOTOR_LOWER_LIMIT_POS;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberConfig.Slot0.kP = 0.3;
        climberConfig.Slot0.kI = 0.005;
        climberConfig.Slot0.kD = 0.0;
        climberConfig.Slot0.kG = 0.3;

        climberConfig.Slot1.kP = 0.3;
        climberConfig.Slot1.kI = 0.005;
        climberConfig.Slot1.kD = 0.0;
        climberConfig.Slot1.kG = 0.25;

        motor1.getConfigurator().apply(climberConfig);
        motor2.getConfigurator().apply(climberConfig);
        motor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, false));

        navX = NavX.getNavX();

        elevatorStopper = new Servo(8);

        climbingChain = false;
    }

    public static ClimberSubsystem getInstance() {
        return climberInstance;
    }

    public double getClimberVelocity() {
        return motor1.getVelocity().getValue().doubleValue();
    }

    public double getClimberPos() {
        return motor1.getPosition().getValue();
    }

    public double getSetPos() {
        return climberPositionVoltage.Position;
    }

    public void runElevatorUp(double speed) {
        speed = Math.abs(speed);
        runElevator(speed);
    }

    public void runElevatorDown(double speed) {
        speed = -Math.abs(speed);
        runElevator(speed);
    }

    public void prepForClimb() {
        climberPositionVoltage.Position = ELEVATOR_MOTOR_UPPER_LIMIT_POS;
//        climbingChain = false;
    }

    public void doClimb() {
        climberPositionVoltage.Position = ELEVATOR_MOTOR_LOWER_LIMIT_POS;
    }

    private void runElevator(double speed) {

        climberVelocity = new VelocityVoltage(ELEVATOR_MAX_ROTATIONS_PER_SECOND * speed);

        climberVelocity.FeedForward = climberFeedForward.calculate(climberVelocity.Velocity);
        //shooterVelocity.withAcceleration(25);

        motor1.setControl(climberVelocity);
    }

    public void stopElevator() {
        motor1.stopMotor();
    }

    public boolean isOnChain() {
        return climbingChain;
    }

    public Command prepForClimbCommand() {
        return (this.servoOut().andThen(Commands.waitSeconds(1))).onlyIf(() -> elevatorStopper.get() < 1)
                .andThen(new InstantCommand(this::prepForClimb, this));
    }

    public Command doClimbClimbCommand() {
        return this.runOnce(
                this::doClimb);
    }

    public Command runClimberUpCommand() {

        return this.run(
                () -> runElevatorUp(0.03)).finallyDo(this::stopElevator);
    }


    public Command runClimberDownCommand() {

        return this.run(
                () -> runElevatorDown(-0.03)).finallyDo(this::stopElevator);
    }

    public Command stopClimberCommand() {

        return this.run(
                this::stopElevator);

    }

    public Command servoOut() {
        return new InstantCommand(
                () -> elevatorStopper.set(1),
                this
        );
    }

    public Command servoIn() {
        return new InstantCommand(
                () -> elevatorStopper.set(200d / 270d)
        );
    }

    @Override
    public void periodic() {
        // Update the set position while disabled
        if (RobotState.isDisabled()) {
            climberPositionVoltage.Position = motor1.getPosition().getValue();
        }

        // Decide if we are climbing on the chain
        if (!climbingChain && climberPositionVoltage.Position == ELEVATOR_MOTOR_LOWER_LIMIT_POS &&
                motor1.getSupplyCurrent().getValue() > 1.7 && motor1.getPosition().getValue() < 4 && motor1.getPosition().getValue() > 2 && motor1.getVelocity().getValue() < 0.1) {
            climbingChain = true;
        }
        else if (climberPositionVoltage.Position != ELEVATOR_MOTOR_LOWER_LIMIT_POS && motor1.getPosition().getValue() > 12) {
            climbingChain = false;
        }

        if (climbingChain) {
            if (climberPositionVoltage.Position == ELEVATOR_MOTOR_LOWER_LIMIT_POS) {
                climberPositionVoltage.FeedForward = -1.5 * MathUtil.clamp((Math.abs(motor1.getClosedLoopError().getValue())) * 100, 0, 10) / 10; //TODO: Tune this feedforward value (in volts)
            } else {
                climberPositionVoltage.FeedForward = 0;//-12 * motor1.getClosedLoopError().getValue()/29;
            }
        } else {
            climberPositionVoltage.FeedForward = 0;
        }

        SmartDashboard.putBoolean("Climber On Chain", climbingChain);
        SmartDashboard.putNumber("Climber Feedforward", climberPositionVoltage.FeedForward);
        SmartDashboard.putNumber("Climber Current", motor1.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Climber Set Position", climberPositionVoltage.Position);
        SmartDashboard.putNumber("Climber Position", motor1.getPosition().getValue());
        SmartDashboard.putNumber("Climber Output", motor1.getClosedLoopOutput().getValue());
        SmartDashboard.putNumber("Climber Speed", motor1.getVelocity().getValue());
        SmartDashboard.putNumber("Climber Closed Loop Error", motor1.getClosedLoopError().getValue());

        SmartDashboard.putNumber("Servo angle", elevatorStopper.get());

        SmartDashboard.putNumber("Altitude", navX.getAltitude());
        SmartDashboard.putBoolean("Altitude Valid", navX.isAltitudeValid());
        SmartDashboard.putNumber("Z accel", navX.getWorldLinearAccelZ());
        SmartDashboard.putNumber("Z displacement", navX.getDisplacementZ());

        if (climbingChain && climberPositionVoltage.Position == ELEVATOR_MOTOR_UPPER_LIMIT_POS && motor1.getPosition().getValue() < 12) {
            climberVelocity.Velocity = 22;
            climberVelocity.FeedForward = -0.25;//climberFeedForward.calculate(climberVelocity.Velocity);
            motor1.setControl(climberVelocity);
            System.out.println("Using Duty Cycle!!");
        } else {
            motor1.setControl(climberPositionVoltage);
        }
    }
}
