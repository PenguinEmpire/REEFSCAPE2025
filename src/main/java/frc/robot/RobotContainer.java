package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.AlgaeHolderSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final PS5Controller ps5Controller = new PS5Controller(0);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final AlgaeHolderSubsystem algaeHolderSubsystem = new AlgaeHolderSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final SwerveDriveCommand swerveDriveCommand = 
        new SwerveDriveCommand(driveSubsystem, ControlInput.forPS5Controller(ps5Controller));

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        logSubsystems(); // Prevents "unused variable" warnings by sending data to SmartDashboard
    }

    private void configureDefaultCommands() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    private void configureButtonBindings() {
        driveSubsystem.setDefaultCommand(
            new SwerveDriveCommand(
                driveSubsystem, 
                ControlInput.forPS5Controller(ps5Controller) // Ensure controller input is passed
            )
        );
    }

    public Command getAutonomousCommand() {
        return null; 
    }

    public void teleopInit() {
        System.out.println("Teleop mode initialized!");
    }

    private void logSubsystems() {
        SmartDashboard.putData("DriveSubsystem", driveSubsystem);
        SmartDashboard.putData("AlgaeHolderSubsystem", algaeHolderSubsystem);
        SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
        SmartDashboard.putData("IntakeSubsystem", intakeSubsystem);
        SmartDashboard.putData("ShooterSubsystem", shooterSubsystem);
    }
}
