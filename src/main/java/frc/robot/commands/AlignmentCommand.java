package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.ControlInput;

public class AlignmentCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private ControlInput controlInput;

    public AlignmentCommand (DriveSubsystem driveSubsystem, ControlInput controlInput){
        this.driveSubsystem = driveSubsystem;
        this.controlInput = controlInput;
        addRequirements(this.driveSubsystem);
        setName("Alignment Command");
    }

    @Override

    
    public void initialize() {
        this.driveSubsystem.getNavX().setAngleAdjustment(0);
        this.driveSubsystem.resetGyroscope();
    }

    @Override
    public void execute(){
        this.driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }


    
}
