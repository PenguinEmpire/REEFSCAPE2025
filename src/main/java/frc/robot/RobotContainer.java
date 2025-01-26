// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.AlgaeHolderSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Controllers
    private final PS4Controller ps5Controller = new PS4Controller(0);

    // Subsystems
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final AlgaeHolderSubsystem algaeHolderSubsystem = new AlgaeHolderSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // Commands
    private final SwerveDriveCommand swerveDriveCommand = 
        new SwerveDriveCommand(driveSubsystem, ps5Controller);

    public RobotContainer() {
        // Configure default commands
        configureDefaultCommands();

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        
        driveSubsystem.setDefaultCommand(swerveDriveCommand);

    }

    private void configureButtonBindings() {
       
    }

    public Command getAutonomousCommand() {   
        return null; 
    }
}
 