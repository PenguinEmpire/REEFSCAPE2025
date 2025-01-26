package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake; 

public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkMax horizontalRollerMotor;
    private final CANSparkMax leftVerticalRollerMotor;
    private final CANSparkMax rightVerticalRollerMotor;
    private final CANSparkMax rotationMotor;

    // REV Bore Encoder for rotation
    private final DutyCycleEncoder boreEncoder;

    // PID Coefficients
    private double rotationP = Intake.ROTATION_P;
    private double rotationI = Intake.ROTATION_I;
    private double rotationD = Intake.ROTATION_D;

    // Roller power
    private double rollerPower = Intake.DEFAULT_ROLLER_POWER;

    public IntakeSubsystem() {
        // Initialize motors
        horizontalRollerMotor = createMotor(Intake.HORIZONTAL_ROLLER_MOTOR_ID);
        leftVerticalRollerMotor = createMotor(Intake.LEFT_VERTICAL_ROLLER_MOTOR_ID);
        rightVerticalRollerMotor = createMotor(Intake.RIGHT_VERTICAL_ROLLER_MOTOR_ID);
        rotationMotor = createMotor(Intake.ROTATION_MOTOR_ID);

        // Initialize REV Bore Encoder
        boreEncoder = new DutyCycleEncoder(Intake.BORE_ENCODER_CHANNEL);
        boreEncoder.setDistancePerRotation(360.0); // Encoder reports position in degrees

        //  SmartDashboard sliders
        SmartDashboard.putNumber("Target Angle", 0.0); // For rotation testing
        SmartDashboard.putNumber("Roller Power", Intake.DEFAULT_ROLLER_POWER); // For roller testing
        SmartDashboard.putNumber("Rotation PID P", rotationP);
        SmartDashboard.putNumber("Rotation PID I", rotationI);
        SmartDashboard.putNumber("Rotation PID D", rotationD);
    }

    private CANSparkMax createMotor(int motorID) {
        CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Intake.MOTOR_CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);
        return motor;
    }

    public void rotateToPosition(boolean extended) {
        double targetAngle = extended ? Intake.ROTATION_MAX_ANGLE : 0.0;
        double currentAngle = getArmAngle();

        // Reset encoder when fully retracted
        if (!extended && Math.abs(currentAngle) <= Intake.ROTATION_TOLERANCE) {
            boreEncoder.reset(); // Reset encoder to zero
            rotationMotor.set(0.0); // Stop the motor
            return;
        }

        // Calculate PID output
        double error = targetAngle - currentAngle;
        double pidOutput = calculatePID(error);

        // Set motor output
        rotationMotor.set(pidOutput);
    }

    private double calculatePID(double error) {
        rotationP = SmartDashboard.getNumber("Rotation PID P", rotationP);
        rotationI = SmartDashboard.getNumber("Rotation PID I", rotationI);
        rotationD = SmartDashboard.getNumber("Rotation PID D", rotationD);

        double output = rotationP * error; // Basic P control
        return Math.max(-1.0, Math.min(1.0, output)); // Clamp output to [-1, 1]
    }

    public void spinRollers(boolean intake) {
        rollerPower = SmartDashboard.getNumber("Roller Power", rollerPower);

        double power = intake ? rollerPower : -rollerPower;
        horizontalRollerMotor.set(power);
        leftVerticalRollerMotor.set(-power);
        rightVerticalRollerMotor.set(power);
    }

    public void stopAllRollers() {
        horizontalRollerMotor.set(0.0);
        leftVerticalRollerMotor.set(0.0);
        rightVerticalRollerMotor.set(0.0);
        rotationMotor.set(0.0);
    }

    public double getArmAngle() {
        return boreEncoder.getDistance(); // angle in degrees
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Arm Angle", getArmAngle());
    }
}
