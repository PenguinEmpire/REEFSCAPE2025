package frc.robot.commands;

import frc.robot.ControlInput;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

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
        double forward = linearDeadband(controlInput.getForward());
        double strafe = linearDeadband(controlInput.getStrafe());
        double rotation = linearDeadband(controlInput.getRotation());

        
        forward = clamp(forward, -1.0, 1.0);
        strafe = clamp(strafe, -1.0, 1.0);
        rotation = clamp(rotation, -1.0, 1.0);

        forward = Math.copySign(forward * forward, forward) * Drive.MAX_SPEED;
        strafe = Math.copySign(strafe * strafe, strafe) * Drive.MAX_SPEED;
        rotation = Math.copySign(rotation * rotation, rotation) * Drive.MAX_ANGULAR_SPEED;

        subsystem.drive(forward, strafe, rotation, true, false);
    }

    private double linearDeadband(double input) {
        final double deadband = 0.095;
        if (Math.abs(input) < deadband) {
            return 0;
        } else {
            return input - Math.copySign(deadband, input);
        }
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
