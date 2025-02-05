package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final ControlInput controlInput = new ControlInput(0); 
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final SwerveDriveCommand swerveDriveCommand;

    public RobotContainer() {
        swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, controlInput);
        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    public void teleopInit() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    public Command getAutonomousCommand() {
        return null; 
}


}
