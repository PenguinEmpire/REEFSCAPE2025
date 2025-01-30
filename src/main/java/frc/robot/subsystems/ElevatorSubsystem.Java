package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final DigitalInput limitSwitch;

    private final PositionVoltage positionControl = new PositionVoltage(0);
    private double kP = Elevator.KP;
    private double kI = Elevator.KI;
    private double kD = Elevator.KD;

    public ElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(Elevator.LEFT_MOTOR_ID);
        rightElevatorMotor = new TalonFX(Elevator.RIGHT_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;

        leftElevatorMotor.getConfigurator().apply(config);
        rightElevatorMotor.getConfigurator().apply(config);

        rightElevatorMotor.setControl(positionControl.withPosition(leftElevatorMotor.getPosition().getValueAsDouble()));

        limitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_CHANNEL);

        SmartDashboard.putNumber("Elevator kP", kP);
        SmartDashboard.putNumber("Elevator kI", kI);
        SmartDashboard.putNumber("Elevator kD", kD);
        SmartDashboard.putNumber("Target Position (meters)", 0.0);
    }

    public void moveToPosition(double targetPosition) {
        double targetTicks = targetPosition / Elevator.ENCODER_CONVERSION_FACTOR;
        leftElevatorMotor.setControl(positionControl.withPosition(targetTicks));
        rightElevatorMotor.setControl(positionControl.withPosition(targetTicks));
    }

    public void moveToIntakePosition() {
        moveToPosition(Elevator.INTAKE_POSITION);
    }

    public void moveToReefLevel1() {
        moveToPosition(Elevator.REEF_LEVEL_1);
    }

    public void moveToReefLevel2() {
        moveToPosition(Elevator.REEF_LEVEL_2);
    }

    public void moveToReefLevel3() {
        moveToPosition(Elevator.REEF_LEVEL_3);
    }

    public void moveToReefLevel4() {
        moveToPosition(Elevator.REEF_LEVEL_4);
    }

    public void stop() {
        leftElevatorMotor.set(0.0);
        rightElevatorMotor.set(0.0);
    }

    public boolean isAtBottom() {
        return !limitSwitch.get();
    }

    public boolean isInIntakePosition() {
        double currentHeight = getHeight();
        return Math.abs(currentHeight - Elevator.INTAKE_POSITION) <= Elevator.TOLERANCE;
    }

    public boolean isFullyExtended() {
        double currentHeight = getHeight();
        return Math.abs(currentHeight - Elevator.REEF_LEVEL_4) <= Elevator.TOLERANCE;
    }

    public void resetEncoder() {
        leftElevatorMotor.setPosition(0);
        rightElevatorMotor.setPosition(0);
    }

    public double getHeight() {
        return leftElevatorMotor.getPosition().getValueAsDouble() * Elevator.ENCODER_CONVERSION_FACTOR;
    }

    @Override
    public void periodic() {
        if (isAtBottom()) {
            resetEncoder();
        }

        double newP = SmartDashboard.getNumber("Elevator kP", kP);
        double newI = SmartDashboard.getNumber("Elevator kI", kI);
        double newD = SmartDashboard.getNumber("Elevator kD", kD);

        if (newP != kP || newI != kI || newD != kD) {
            kP = newP;
            kI = newI;
            kD = newD;

            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Slot0.kP = kP;
            config.Slot0.kI = kI;
            config.Slot0.kD = kD;

            leftElevatorMotor.getConfigurator().apply(config);
            rightElevatorMotor.getConfigurator().apply(config);
        }

        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
        SmartDashboard.putBoolean("In Intake Position", isInIntakePosition());
        SmartDashboard.putBoolean("Fully Extended", isFullyExtended());
    }
}
