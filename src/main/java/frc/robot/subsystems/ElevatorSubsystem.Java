package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // replace with actual ids
    private static final int LEFT_MOTOR_ID = 9; 
    private static final int RIGHT_MOTOR_ID = 10; 

    // Motors
    private final CANSparkMax leftElevatorMotor;
    private final CANSparkMax rightElevatorMotor;

    // PID Controller
    private final SparkMaxPIDController pidController;

    // Encoder
    private final RelativeEncoder leftMotorEncoder;

    // Limit switch
    private final DigitalInput limitSwitch;

    // Constants need to be moved over once done
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
        leftElevatorMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        rightElevatorMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

        // Configure motors
        leftElevatorMotor.restoreFactoryDefaults();
        rightElevatorMotor.restoreFactoryDefaults();
        rightElevatorMotor.follow(leftElevatorMotor); // Synchronize motors

        // Initialize the encoder from the left motor
        leftMotorEncoder = leftElevatorMotor.getEncoder();
        leftMotorEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

        // Initialize PID Controller
        pidController = leftElevatorMotor.getPIDController();
        pidController.setFeedbackDevice(leftMotorEncoder);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);

        //replace with correct channel
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
        pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
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
        leftMotorEncoder.setPosition(0.0);
    }

    /**
     * Gets the current height of the elevator in meters.
     * 
     * @return The elevator height.
     */
    public double getHeight() {
        return leftMotorEncoder.getPosition();
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
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
        }

        // Update SmartDashboard with elevator status
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
    }
}
