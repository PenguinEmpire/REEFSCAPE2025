// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSubsystem;
    private final PS4Controller ps5Controller;

    public RobotContainer() {
        // Initialize subsystems
        driveSubsystem = new DriveSubsystem();

        // Initialize the PS4 controller on port 0
        ps5Controller = new PS4Controller(0);

        // Configure default commands
        driveSubsystem.setDefaultCommand(new SwerveDriveCommand(driveSubsystem, ps5Controller));
    }
}
