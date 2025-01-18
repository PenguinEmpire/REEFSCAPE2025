// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.ControlInput;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    private DriveSubsystem subsystem;
    private PIDController autoAlignPID;
    private ControlInput input;
    private double targetRot;
    private PIDController xPID;
    private PIDController yPID;

    private boolean wasTargeting = false;

    
  //todo: add vision and lighting
  public SwerveDriveCommand(DriveSubsystem subsystem, ControlInput ci) {
    this.subsystem = subsystem;
    this.input = ci;
    this.targetRot = subsystem.getAngle();
    addRequirements(subsystem);
    setName("SwerveDrive");
    autoAlignPID = new PIDController(0.0154, 0, 0);
    xPID = new PIDController(0.02, 0, 0);
    yPID = new PIDController(0.02, 0, 0);
    SmartDashboard.putBoolean("Inverted Pickup", false);

  }

  @Override
  public void initialize() {
    autoAlignPID.enableContinuousInput(0, 360);
    xPID.reset();
    yPID.reset();
    wasTargeting = false;
  }

  @Override
  public void execute() {
    double forward = -getInput().getLeftJoystick().getRawAxis(1);
    double pow = 2;
    forward = linearDeadband(forward);
    
    forward = Math.copySign(Math.pow(forward, pow), forward);
   
    double strafe = -getInput().getLeftJoystick().getRawAxis(0);
    strafe = linearDeadband(strafe);
  
    strafe = Math.copySign(Math.pow(strafe, pow), strafe);
   
    double rotation = getInput().getRightJoystick().getRawAxis(2);

    rotation = Math.copySign(Math.pow(rotation, pow), rotation);
    rotation = linearDeadband(rotation);

    // double isFacingOtherSide = std::abs(m_subsystem->GetAngle().value()) > 90 && std::abs(m_subsystem->GetAngle().value()) < 270;
    //  if(isFacingOtherSide) {
    //         m_vs->SetCustomPipeline(true);
    //         bool isInverted = SmartDashboard.GetBoolean("Inverted Pickup", false);
    //         if(!isInverted) m_vs->SetPipeline(2);
    //         else m_vs->SetPipeline(3);
    //   }
    //   else {
    //       m_vs->SetCustomPipeline(false);
    //   }
    // if(getInput().getLeftJoystick().getTrigger()) {
      // double angleToGoalDegrees = physical::vision::limelightMountAngleDegrees + m_vs->GetPitch();
      // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      // double distanceFromLimelightToGoalInches = (physical::vision::goalHeightInches - physical::vision::limelightLensHeightInches)/tan(angleToGoalRadians);
      // if(SmartDashboard.GetBoolean("Enable Debug", false)) SmartDashboard.PutNumber("target dist", distanceFromLimelightToGoalInches);
      // if(m_input->IsConeMode()) m_vs->SetLED(true);
      // double test = 0;
      // if(isFacingOtherSide) {
      //     test = 180;
      //     forward *= -1;
      // }
      // double turnValue = -autoAlignPID.calculate(m_subsystem.getAngle().value(), test);
      // if(!m_vs->HasTargets()) {
      //     if(m_wasTargeting) {
      //         m_subsystem->Drive(
      //             units::meters_per_second_t(forward * 0.4),
      //             units::meters_per_second_t(0), 
      //             units::radians_per_second_t{std::clamp(std::clamp(turnValue, -0.4, 0.4), -physical::drive::kMaxAngularVelocity,physical::drive::kMaxAngularVelocity)}, false, false// keep rot static
      //         );
      //     }
      //     return;
      // }
      // wasTargeting = true;
      // m_ls->SetTemporaryColor(0,255,0);

      // strafe = -clamp(yPID->Calculate(m_vs->GetYaw(), 0), -0.3, 0.3);
 
  //     subsystem.drive(
  //         forward * 0.4,
  //         strafe, 
  //         clamp(clamp(turnValue, -0.4, 0.4), -Drive.kMaxAngularVelocity), false, false);
  // }
    // if(m_input->IsConeMode()) m_vs->SetLED(false);
        subsystem.drive(forward, strafe, clamp(rotation * 0.8, -Drive.MAX_ANGULAR_VELOCITY, Drive.MAX_ANGULAR_VELOCITY), true, false);
        // autoAlignPID.reset();
        // yPID.reset();
  }

  private double linearDeadband(double input) {
    final double deadband = 0.095;
    if(Math.abs(input) < deadband) {
      return 0;
    }
    else {
      return input - deadband;
    }
  }

  public ControlInput getInput() {
    return this.input;
  }

  public double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}