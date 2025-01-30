package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.ControlInput;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDriveCommand extends Command {
    private final DriveSubsystem subsystem;
    private final ControlInput controlInput;

    private final PIDController rotationPID;
    private final PIDController forwardPID;
    private final PIDController strafePID;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    public SwerveDriveCommand(DriveSubsystem subsystem, ControlInput controlInput) {
        this.subsystem = subsystem;
        this.controlInput = controlInput;

        addRequirements(subsystem);
        setName("SwerveDrive");

        rotationPID = new PIDController(0.015, 0.001, 0.0001);
        forwardPID = new PIDController(0, 0, 0);
        strafePID = new PIDController(0, 0, 0);

        SmartDashboard.putNumber("Rotation PID P", rotationPID.getP());
        SmartDashboard.putNumber("Rotation PID I", rotationPID.getI());
        SmartDashboard.putNumber("Rotation PID D", rotationPID.getD());
        SmartDashboard.putNumber("Joystick Deadband", 0.095);
        SmartDashboard.putBoolean("Field Relative", true);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        forwardPID.reset();
        strafePID.reset();
    }

    @Override
    public void execute() {
        rotationPID.setP(SmartDashboard.getNumber("Rotation PID P", rotationPID.getP()));
        rotationPID.setI(SmartDashboard.getNumber("Rotation PID I", rotationPID.getI()));
        rotationPID.setD(SmartDashboard.getNumber("Rotation PID D", rotationPID.getD()));

        double deadband = SmartDashboard.getNumber("Joystick Deadband", 0.095);
        double forward = applyDeadbandAndScale(controlInput.getForward(), 2.0, deadband);
        double strafe = applyDeadbandAndScale(controlInput.getStrafe(), 2.0, deadband);
        double rotation = applyDeadbandAndScale(controlInput.getRotation(), 2.0, deadband);

        forward = forwardLimiter.calculate(forward * Drive.MAX_SPEED);
        strafe = strafeLimiter.calculate(strafe * Drive.MAX_SPEED);
        rotation = rotationLimiter.calculate(rotation * Drive.MAX_ANGULAR_SPEED);

        double smoothedRotation = rotationPID.calculate(rotation, 0.0);
        boolean fieldRelative = SmartDashboard.getBoolean("Field Relative", true);

        subsystem.drive(forward, strafe, smoothedRotation, fieldRelative);

        SmartDashboard.putNumber("Forward Input", forward);
        SmartDashboard.putNumber("Strafe Input", strafe);
        SmartDashboard.putNumber("Rotation Input", rotation);
        SmartDashboard.putNumber("Smoothed Rotation", smoothedRotation);
    }

    private double applyDeadbandAndScale(double input, double power, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        }
        return Math.copySign(Math.pow(input, power), input);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
