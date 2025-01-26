package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeHolder;

public class AlgaeHolderSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkMax pivotMotor;
    private final CANSparkMax rollingMotor;

    // Encoder for pivot motor
    private final DutyCycleEncoder pivotEncoder;

    // Limit switch
    private final DigitalInput limitSwitch;

    public AlgaeHolderSubsystem() {
        // Initialize pivot motor
        pivotMotor = new CANSparkMax(AlgaeManipulator.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        // Initialize rolling motor
        rollingMotor = new CANSparkMax(AlgaeManipulator.ROLLING_MOTOR_ID, MotorType.kBrushless);
        rollingMotor.restoreFactoryDefaults(); 

        // Initialize pivot encoder
        pivotEncoder = new DutyCycleEncoder(AlgaeManipulator.LIMIT_SWITCH_CHANNEL);
        pivotEncoder.setDistancePerRotation(360.0); // Encoder reports angle in degrees

        // Initialize limit switch
        limitSwitch = new DigitalInput(AlgaeManipulator.LIMIT_SWITCH_CHANNEL);

        //PID tuning and positions to SmartDashboard
        SmartDashboard.putNumber("Pivot P", AlgaeManipulator.PIVOT_P);
        SmartDashboard.putNumber("Pivot I", AlgaeManipulator.PIVOT_I);
        SmartDashboard.putNumber("Pivot D", AlgaeManipulator.PIVOT_D);
        SmartDashboard.putNumber("Rolling Power", AlgaeManipulator.ROLLING_POWER);
    }

    public void setPivotPosition(double targetAngle) {
        // logic to move pivot motor to the target angle
        SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    }

    public void rollMotor(boolean intake) {
        double power = SmartDashboard.getNumber("Rolling Power", AlgaeManipulator.ROLLING_POWER);
        rollingMotor.set(intake ? power : -power);
    }

    public void stopRollingMotor() {
        rollingMotor.set(0.0);
    }

    public boolean isLimitSwitchTriggered() {
        return !limitSwitch.get(); // Assuming the limit switch is normally open
    }

    @Override
    public void periodic() {
        // Stop rolling motor if the limit switch is triggered
        if (isLimitSwitchTriggered()) {
            stopRollingMotor();
            SmartDashboard.putBoolean("Limit Switch Triggered", true);
        } else {
            SmartDashboard.putBoolean("Limit Switch Triggered", false);
        }

        // Display current pivot angle on SmartDashboard
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getDistance());
    }
}
