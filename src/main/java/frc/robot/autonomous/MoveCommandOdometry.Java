package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveCommandOdometry extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPosition;
    private final double maxSpeed;

    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController turnPID;

    private int ticks;

    public MoveCommandOdometry(DriveSubsystem driveSubsystem, Pose2d targetPosition, double maxSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.targetPosition = targetPosition;
        this.maxSpeed = maxSpeed;

        // PID controllers for X, Y, and rotation
        xPID = new PIDController(3.0, 0.0, 0.0);
        yPID = new PIDController(3.0, 0.0, 0.0);
        turnPID = new PIDController(0.04, 0.0, 0.0);

        
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset PID controllers
        xPID.reset();
        yPID.reset();
        turnPID.reset();

        // Set tolerances for position and rotation
        xPID.setTolerance(0.02); // Meters
        yPID.setTolerance(0.02); // Meters
        turnPID.setTolerance(1.0); // Degrees
        turnPID.enableContinuousInput(-180, 180);

        ticks = 0; // Initialize tick counter
    }

    @Override
    public void execute() {
        ticks++;

        // Get the current position and orientation of the robot
        Pose2d currentPose = driveSubsystem.getPosition();
        Translation2d currentTranslation = currentPose.getTranslation();
        double currentAngle = currentPose.getRotation().getDegrees();

        // Calculate control outputs for X, Y, and rotation
        double xOutput = xPID.calculate(currentTranslation.getX(), targetPosition.getTranslation().getX());
        double yOutput = yPID.calculate(currentTranslation.getY(), targetPosition.getTranslation().getY());
        double turnOutput = turnPID.calculate(currentAngle, targetPosition.getRotation().getDegrees());

        // Apply gradual ramp-up for smooth motion
        double rampFactor = Math.min(ticks / 20.0, 1.0); // Ramp up over 20 ticks
        double maxDrive = maxSpeed * rampFactor;
        double maxTurn = 0.4 * rampFactor;

        // Drive the robot using PID outputs
        driveSubsystem.drive(
            clamp(xOutput, maxDrive),
            clamp(yOutput, maxDrive),
            clamp(turnOutput, maxTurn),
            true // Field-relative control
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        // Check if the robot has reached the target position and orientation
        boolean positionReached = xPID.atSetpoint() && yPID.atSetpoint();
        boolean rotationReached = turnPID.atSetpoint();
        return positionReached && rotationReached;
    }

    // Clamp a value to a specific range
    private double clamp(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
}
