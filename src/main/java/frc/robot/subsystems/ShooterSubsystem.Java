package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final DigitalInput limitSwitch;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(Shooter.MOTOR_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(Shooter.LIMIT_SWITCH_CHANNEL);

        SmartDashboard.putNumber("Shooter Power", Shooter.FORWARD_POWER);
    }

    public void shootForward() {
        double power = SmartDashboard.getNumber("Shooter Power", Shooter.FORWARD_POWER);
        shooterMotor.set(power);
    }

    public void shootReverse() {
        shooterMotor.set(Shooter.REVERSE_POWER);
    }

    public void stopShooter() {
        shooterMotor.set(Shooter.STOP_POWER);
    }

    public boolean isLimitSwitchTriggered() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        if (isLimitSwitchTriggered()) {
            stopShooter();
            SmartDashboard.putBoolean("Shooter Limit Switch Triggered", true);
        } else {
            SmartDashboard.putBoolean("Shooter Limit Switch Triggered", false);
        }
        SmartDashboard.putNumber("Shooter Current Power", shooterMotor.get());
    }
}
