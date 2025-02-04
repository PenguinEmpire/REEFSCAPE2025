package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final PS5Controller ps5Controller = new PS5Controller(0);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private SwerveDriveCommand swerveDriveCommand;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, ControlInput.forPS5Controller(ps5Controller));
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    public void teleopInit() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
