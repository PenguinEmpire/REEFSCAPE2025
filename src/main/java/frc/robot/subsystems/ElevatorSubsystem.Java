package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase {
    // Replace with actual ids
    private static final int LEFT_MOTOR_ID = 9; 
    private static final int RIGHT_MOTOR_ID = 10; 

    // Motors
    private final WPI_TalonFX leftElevatorMotor;
    private final WPI_TalonFX rightElevatorMotor;

    // Limit switch
    private final DigitalInput limitSwitch;

    // Constants
    private static final double ENCODER_CONVERSION_FACTOR = 0.01; // Meters per revolution
    private static final double TOLERANCE = 0.05; // Tolerance for position checks in meters

    // Preset positions for two stages (in meters)
    private static final double BOTTOM_POSITION = 0.0; // Bottom position
    private static final double MID_STAGE_POSITION = 1.0; // Mid-stage
    private static final double TOP_STAGE_POSITION = 2.0; // Top-stage

    // PID Coefficients
    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;

    public ElevatorSubsystem() {
        // Initialize motors
        leftElevatorMotor = new WPI_TalonFX(LEFT_MOTOR_ID);
        rightElevatorMotor = new WPI_TalonFX(RIGHT_MOTOR_ID);

        // Configure motors
        rightElevatorMotor.follow(leftElevatorMotor); // Synchronize motors

        // Initialize PID coefficients for Kraken X60 (using TalonFX)
        leftElevatorMotor.config_kP(0, kP);
        leftElevatorMotor.config_kI(0, kI);
        leftElevatorMotor.config_kD(0, kD);

        // Initialize limit switch (replace with correct channel)
        limitSwitch = new DigitalInput(0); 

        // Reset encoder at initialization
        resetEncoder();

        // Add PID values to SmartDashboard
        SmartDashboard.putNumber("Elevator kP", kP);
        SmartDashboard.putNumber("Elevator kI", kI);
        SmartDashboard.putNumber("Elevator kD", kD);
    }

    /**
     * Moves the elevator to a target position using PID control.
     * 
     * @param targetPosition The desired position in meters.
     */
    public void moveToPosition(double targetPosition) {
        // Convert target position to encoder ticks
        double targetTicks = targetPosition / ENCODER_CONVERSION_FACTOR;

        // Set the target position using TalonFX's set() method with PID control
        leftElevatorMotor.set(ControlMode.Position, targetTicks);
    }

    public void moveToBottom() {
        moveToPosition(BOTTOM_POSITION);
    }

    public void moveToMidStage() {
        moveToPosition(MID_STAGE_POSITION);
    }

    public void moveToTopStage() {
        moveToPosition(TOP_STAGE_POSITION);
    }

    public void stop() {
        leftElevatorMotor.set(0.0);
    }

    /**
     * Checks if the limit switch is pressed (elevator is at the bottom).
     * 
     * @return True if at the bottom
     */
    public boolean isAtBottom() {
        return !limitSwitch.get(); // Assuming the limit switch is normally open
    }

    /**
     * Resets the encoder to zero.
     */
    public void resetEncoder() {
        leftElevatorMotor.setSelectedSensorPosition(0); 
    }

    /**
     * Gets the current height of the elevator in meters.
     * 
     * @return The elevator height.
     */
    public double getHeight() {
        return leftElevatorMotor.getSelectedSensorPosition() * ENCODER_CONVERSION_FACTOR;
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
            // Update the PID coefficients on the motor
            leftElevatorMotor.config_kP(0, kP);
            leftElevatorMotor.config_kI(0, kI);
            leftElevatorMotor.config_kD(0, kD);
        }

        // Update SmartDashboard with elevator status
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
    }
}
