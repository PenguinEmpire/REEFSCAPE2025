package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Motor CAN IDs ( change later)   
    private static final int ROTATION_MOTOR_ID = 12;
    private static final int MAIN_ROLLER_MOTOR_ID = 13;
    private static final int LEFT_WHEEL_MOTOR_ID = 14;
    private static final int RIGHT_WHEEL_MOTOR_ID = 15;

    // Infrared sensor channel ( should be the same as the shooter)
    private static final int INFRARED_SENSOR_CHANNEL = 0;

    // Bore Encoder DIO channel( change to actual channel)
    private static final int BORE_ENCODER_CHANNEL = 1;

    // Motors
    private final CANSparkMax rotationMotor;
    private final CANSparkMax mainRollerMotor;
    private final CANSparkMax leftWheelMotor;
    private final CANSparkMax rightWheelMotor;

    // Bore Encoder for arm angle
    private final DutyCycleEncoder boreEncoder;

    // PID coefficients for rotation motor
    private double rotationP = 0.1;
    private double rotationI = 0.0;
    private double rotationD = 0.0;

    // Rotation constants
    private static final double ROTATION_MAX_ANGLE = 200.0; // Max angle in degrees
    private static final double ROTATION_TOLERANCE = 5.0;   // Tolerance in degrees

    // Default roller power 
    private double rollerPower = 0.8;

    public IntakeSubsystem() {
        // Initialize motors
        rotationMotor = new CANSparkMax(ROTATION_MOTOR_ID, MotorType.kBrushless);
        mainRollerMotor = new CANSparkMax(MAIN_ROLLER_MOTOR_ID, MotorType.kBrushless);
        leftWheelMotor = new CANSparkMax(LEFT_WHEEL_MOTOR_ID, MotorType.kBrushless);
        rightWheelMotor = new CANSparkMax(RIGHT_WHEEL_MOTOR_ID, MotorType.kBrushless);

        // Initialize Bore Encoder
        boreEncoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);
        boreEncoder.setDistancePerRotation(360.0); // Encoder reports position in degrees

        // Add PID coefficients to SmartDashboard
        SmartDashboard.putNumber("Rotation PID P", rotationP);
        SmartDashboard.putNumber("Rotation PID I", rotationI);
        SmartDashboard.putNumber("Rotation PID D", rotationD);

        // Add SmartDashboard control for roller power
        SmartDashboard.putNumber("Roller Power", rollerPower);
    }

    /**
     * Rotates the intake to a specific angle using the Bore Encoder.
     *
     * @param targetAngle The desired angle in degrees (0 to ROTATION_MAX_ANGLE).
     */
    public void rotateToAngle(double targetAngle) {
        if (targetAngle > ROTATION_MAX_ANGLE) {
            targetAngle = ROTATION_MAX_ANGLE;
        } else if (targetAngle < 0) {
            targetAngle = 0;
        }

        // Get current angle from Bore Encoder
        double currentAngle = getArmAngle();

        // Calculate PID output
        double error = targetAngle - currentAngle;
        double pidOutput = calculatePID(error);

        // Set motor output
        rotationMotor.set(pidOutput);
    }

    /**
     * Calculates PID output based on error.
     *
     * @param error The difference between target and current angles.
     * @return The motor output (clamped between -1 and 1).
     */
    private double calculatePID(double error) {
        rotationP = SmartDashboard.getNumber("Rotation PID P", rotationP);
        rotationI = SmartDashboard.getNumber("Rotation PID I", rotationI);
        rotationD = SmartDashboard.getNumber("Rotation PID D", rotationD);

        double output = rotationP * error; 
        return Math.max(-1.0, Math.min(1.0, output)); // Clamp output to [-1, 1]
    }

    /**
     * Spins the main roller and synchronized wheel rollers to intake or outtake coral.
     *
     * @param intake True for intake, false for outtake.
     */
    public void spinRollers(boolean intake) {
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);

        // Spin the main roller
        mainRollerMotor.set(intake ? rollerPower : -rollerPower);

        // Spin the left and right wheel rollers with correct directions
        double wheelPower = intake ? -rollerPower : rollerPower;
        leftWheelMotor.set(wheelPower);
        rightWheelMotor.set(-wheelPower);
    }

    /**
     * Stops all rollers and the rotation motor.
     */
    public void stopAllRollers() {
        mainRollerMotor.set(0.0);
        leftWheelMotor.set(0.0);
        rightWheelMotor.set(0.0);
        rotationMotor.set(0.0);
    }

    /**
     * Gets the current arm angle from the Bore Encoder.
     *
     * @return The current angle in degrees.
     */
    public double getArmAngle() {
        return boreEncoder.getDistance(); // Returns angle in degrees
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with current arm angle and PID tuning
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
    }
}
