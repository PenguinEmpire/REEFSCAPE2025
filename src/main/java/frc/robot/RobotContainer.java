// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.ControlInput;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.AlignmentCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private SwerveDriveCommand swerveDriveCommand;
  private AlignmentCommand alignmentCommand;

  private ControlInput controlInput;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    controlInput = new ControlInput();
    driveSubsystem = new DriveSubsystem();
  }

  private void configureBindings() {
    
  }

  public void teleopInit() {
    swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, controlInput);
    driveSubsystem.setDefaultCommand(swerveDriveCommand);

    alignmentCommand = new AlignmentCommand(driveSubsystem, controlInput);
    JoystickButton alignmentButton  = new JoystickButton(controlInput.getLeftJoystick(),5);
    alignmentButton.whileTrue(alignmentCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}