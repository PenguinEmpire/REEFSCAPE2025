package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    // Controller
    private final PS5Controller ps5Controller = new PS5Controller(0);

    // Subsystems (
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    // Commands
    private SwerveDriveCommand swerveDriveCommand;

    /** The container for the robot. Contains subsystems and input configurations. */
    public RobotContainer() {
        configureBindings();
    }

    /** Configure button bindings and default commands. */
    private void configureBindings() {
        // Set default command for swerve drive
        swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, ControlInput.forPS5Controller(ps5Controller));
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    /** Called at the start of teleop to reset default command. */
    public void teleopInit() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    /** Returns the autonomous command. */
    public Command getAutonomousCommand() {
        return null; // No auto command for now
    }
}
