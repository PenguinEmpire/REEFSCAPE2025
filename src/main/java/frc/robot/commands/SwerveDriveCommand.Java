package frc.robot.commands;

import frc.ControlInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final ControlInput controlInput;

    // PID Controller for smoothing rotational input
    private final PIDController rotationPID;

    public SwerveDriveCommand(DriveSubsystem driveSubsystem, ControlInput controlInput) {
        this.driveSubsystem = driveSubsystem;
        this.controlInput = controlInput;

        // Declare subsystem dependencies
        addRequirements(driveSubsystem);
        setName("SwerveDrive");

        // Initialize rotation PID controller
        rotationPID = new PIDController(0.015, 0.001, 0.0001); // Tunable for rotational smoothing
    }

    @Override
    public void initialize() {
        // Reset rotation PID at the start of the command
        rotationPID.reset();
    }

    @Override
    public void execute() {
        // Get joystick inputs from ControlInput
        double forward = applyDeadbandAndScale(controlInput.getForward(), 2.0, 0.095); // Forward/backward
        double strafe = applyDeadbandAndScale(controlInput.getStrafe(), 2.0, 0.095);   // Left/right strafe
        double rotation = applyDeadbandAndScale(controlInput.getRotation(), 2.0, 0.095); // Rotation

        // Scale inputs to robot's max speed and angular velocity
        forward *= DriveConstants.MAX_SPEED;
        strafe *= DriveConstants.MAX_SPEED;
        rotation *= DriveConstants.MAX_ANGULAR_SPEED;

        // Smooth rotational input with PID
        double smoothedRotation = rotationPID.calculate(rotation, 0.0);

        // Use odometry heading for field-relative control
        driveSubsystem.drive(forward, strafe, smoothedRotation, true);

        // Debugging outputs on SmartDashboard
        SmartDashboard.putNumber("Forward Input", forward);
        SmartDashboard.putNumber("Strafe Input", strafe);
        SmartDashboard.putNumber("Rotation Input", rotation);
        SmartDashboard.putNumber("Smoothed Rotation", smoothedRotation);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted
        return false;
    }

    // Helper method to apply deadband and scale joystick input
    private double applyDeadbandAndScale(double input, double power, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0; // Apply deadband
        }
        return Math.copySign(Math.pow(input, power), input); // Power curve for finer control
    }
}
