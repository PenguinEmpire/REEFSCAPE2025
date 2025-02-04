package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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

    public SwerveDriveCommand(DriveSubsystem subsystem, ControlInput controlInput) {
        this.subsystem = subsystem;
        this.controlInput = controlInput;

        addRequirements(subsystem);
        setName("SwerveDrive");

        rotationPID = new PIDController(0.015, 0.001, 0.0001);
        forwardPID = new PIDController(0, 0, 0);
        strafePID = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        forwardPID.reset();
        strafePID.reset();
    }

    @Override
    public void execute() {
        double deadband = 0.095;

        double forward = applyDeadbandAndScale(controlInput.getForward(), 2.0, deadband);
        double strafe = applyDeadbandAndScale(controlInput.getStrafe(), 2.0, deadband);
        double rotation = applyDeadbandAndScale(controlInput.getRotation(), 2.0, deadband);

        forward *= Drive.MAX_SPEED;
        strafe *= Drive.MAX_SPEED;
        rotation *= Drive.MAX_ANGULAR_SPEED;
        

        double smoothedRotation = rotationPID.calculate(rotation, 0.0);

        // Apply inputs to the swerve drive
        subsystem.drive(forward, strafe, smoothedRotation, true);

        // Debug output to SmartDashboard
        SmartDashboard.putNumber("Gyro Yaw", subsystem.getNavX().getYaw());
        SmartDashboard.putNumber("Gyro Angle", subsystem.getNavX().getAngle());
        SmartDashboard.putNumber("Gyro Heading", subsystem.getHeading());

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
