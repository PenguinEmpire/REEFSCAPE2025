 package frc.robot;

// import edu.wpi.first.wpilibj2.command.Command;  
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.commands.SwerveDriveCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj.PS5Controller;

// public class RobotContainer {
//     private final ControlInput controlInput = new ControlInput(0); 
//     private final DriveSubsystem driveSubsystem = new DriveSubsystem();
//     private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
//     private SwerveDriveCommand swerveDriveCommand; 

//     public RobotContainer() {
//         swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, controlInput);
//         configureBindings();
//     }

//     public void robotInit() {
//         // set angle adjustment on Navx
//         driveSubsystem.getNavX().setAngleAdjustment(0);
//     }

//     private void configureBindings() {
//         driveSubsystem.setDefaultCommand(swerveDriveCommand);

//         // Get PS5 controller instance
//         PS5Controller ps5 = controlInput.getController();

//         // Triangle Button (Button 4) → Intake while held, stop when released( circle)
//         new JoystickButton(ps5, 4).whileTrue(
//             new InstantCommand(() -> intakeSubsystem.spinRollers(true), intakeSubsystem)
//         ).onFalse(
//             new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
//         );

//         // Circle Button (Button 3) → Outtake while held, stop when released (x)
//         new JoystickButton(ps5, 3).whileTrue(
//             new InstantCommand(() -> intakeSubsystem.spinRollers(false), intakeSubsystem)
//         ).onFalse(
//             new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
//         );
//     }

//     public void teleopInit() {
//         swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, controlInput);
//         driveSubsystem.setDefaultCommand(swerveDriveCommand);
//     }

//     public Command getAutonomousCommand() {
//         return null; 
//     }
// }






import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0); 
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private SwerveDriveCommand swerveDriveCommand; 

    public RobotContainer() {
        swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, ps5Controller);
        configureBindings();
    }

    public void robotInit() {
        driveSubsystem.getNavX().setAngleAdjustment(0);
    }


    private void configureBindings() {
        driveSubsystem.setDefaultCommand(swerveDriveCommand);

       
// //triangle for intake
//         ps5Controller.triangle().whileTrue(
//     new RunCommand(() -> intakeSubsystem.spinRollers(true), intakeSubsystem)
//     ) .onFalse(
//     new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
//     ) ;
// // circle for outtake
//          ps5Controller.circle().whileTrue(
//     new RunCommand(() -> intakeSubsystem.spinRollers(false), intakeSubsystem)
//     ).onFalse(
//     new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
//     );


        // triangle for Intake
        ps5Controller.triangle().whileTrue(
            new InstantCommand(() -> intakeSubsystem.spinRollers(true), intakeSubsystem)
        ).onFalse(
            new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
        );

        //circle Outtake
        ps5Controller.circle().whileTrue(
            new InstantCommand(() -> intakeSubsystem.spinRollers(false), intakeSubsystem)
        ).onFalse(
            new InstantCommand(intakeSubsystem::stopAllRollers, intakeSubsystem)
        );
    }

    public void teleopInit() {
        swerveDriveCommand = new SwerveDriveCommand(driveSubsystem, ps5Controller);
        driveSubsystem.setDefaultCommand(swerveDriveCommand);
    }

    public Command getAutonomousCommand() {
        return null; 
    }
}
