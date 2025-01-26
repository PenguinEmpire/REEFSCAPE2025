package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    private final WPI_TalonFX leftElevatorMotor;
    private final WPI_TalonFX rightElevatorMotor;

    // Limit switch
    private final DigitalInput limitSwitch;

    // PID coefficients
    private double kP = Elevator.kP;
    private double kI = Elevator.kI;
    private double kD = Elevator.kD;

    public ElevatorSubsystem() {
        // Initialize motors
        leftElevatorMotor = new WPI_TalonFX(Elevator.LEFT_MOTOR_ID);
        rightElevatorMotor = new WPI_TalonFX(Elevator.RIGHT_MOTOR_ID);

        // Synchronize motors
        rightElevatorMotor.follow(leftElevatorMotor);

        // Configure PID coefficients
        leftElevatorMotor.config_kP(0, kP);
        leftElevatorMotor.config_kI(0, kI);
        leftElevatorMotor.config_kD(0, kD);

        // Initialize limit switch
        limitSwitch = new DigitalInput(Elevator.LIMIT_SWITCH_CHANNEL);

        // Reset encoder at initialization
        resetEncoder();

        // Add PID values and testing sliders to SmartDashboard
        SmartDashboard.putNumber("Elevator kP", kP);
        SmartDashboard.putNumber("Elevator kI", kI);
        SmartDashboard.putNumber("Elevator kD", kD);
        SmartDashboard.putNumber("Target Position (meters)", 0.0);
    }

    public void moveToPosition(double targetPosition) {
        double targetTicks = targetPosition / Elevator.ENCODER_CONVERSION_FACTOR;
        leftElevatorMotor.set(ControlMode.Position, targetTicks);
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
    }

    public boolean isAtBottom() {
        return !limitSwitch.get(); // Assuming the limit switch is normally open
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
        leftElevatorMotor.setSelectedSensorPosition(0);
    }

    public double getHeight() {
        return leftElevatorMotor.getSelectedSensorPosition() * Elevator.ENCODER_CONVERSION_FACTOR;
    }

    @Override
    public void periodic() {
        // Reset encoder if limit switch is triggered
        if (isAtBottom()) {
            resetEncoder();
        }

        // Read PID values from SmartDashboard and update controller
        double newP = SmartDashboard.getNumber("Elevator kP", kP);
        double newI = SmartDashboard.getNumber("Elevator kI", kI);
        double newD = SmartDashboard.getNumber("Elevator kD", kD);

        if (newP != kP || newI != kI || newD != kD) {
            kP = newP;
            kI = newI;
            kD = newD;
            leftElevatorMotor.config_kP(0, kP);
            leftElevatorMotor.config_kI(0, kI);
            leftElevatorMotor.config_kD(0, kD);
        }

        // Update SmartDashboard with elevator status
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
        SmartDashboard.putBoolean("In Intake Position", isInIntakePosition());
        SmartDashboard.putBoolean("Fully Extended", isFullyExtended());


    }
}
