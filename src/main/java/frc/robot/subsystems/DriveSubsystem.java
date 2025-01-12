// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModules;
import frc.robot.drivetrain.SwerveModule;


public class DriveSubsystem extends SubsystemBase {
  
  //field oriented rotation pid constants
  private final double kP = 0.0134f;
  private final double kI = 0.00f;
  private final double kD = 0.00f;
  private final double kF = 0.00f;

  /* This tuning parameter indicates how close to "on target" the    */
  /* PID Controller will attempt to get.                             */

  private final double kToleranceDegrees = 2.0f;

  //current rotation rate
  private double rotateToAngle;
  private PIDController turnController;
  private Pose2d location;
  private int ticks = 0;
  private boolean driveEnabled = true;
  private AHRS navX = new AHRS(Port.kUSB);
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveModules.FRONTLEFT.getModuleLocation(), 
    SwerveModules.FRONTRIGHT.getModuleLocation(),
    SwerveModules.BACKLEFT.getModuleLocation(),
    SwerveModules.BACKRIGHT.getModuleLocation());

  SwerveModule frontLeftModule = new SwerveModule(
    "FrontLeft",
    SwerveModules.FRONTLEFT.getDriveMotorID(),
    SwerveModules.FRONTLEFT.getTurnMotorID(),
    SwerveModules.FRONTLEFT.getEncoderPort(),
    SwerveModules.FRONTLEFT.getModuleLocation(),
    SwerveModules.FRONTLEFT.getEncoderOffset()
  );

  SwerveModule frontRightModule = new SwerveModule(
    "FrontRight",
    SwerveModules.FRONTRIGHT.getDriveMotorID(),
    SwerveModules.FRONTRIGHT.getTurnMotorID(),
    SwerveModules.FRONTRIGHT.getEncoderPort(),
    SwerveModules.FRONTRIGHT.getModuleLocation(),
    SwerveModules.FRONTRIGHT.getEncoderOffset()
  );

  SwerveModule backLeftModule = new SwerveModule(
    "BackLeft",
    SwerveModules.BACKLEFT.getDriveMotorID(),
    SwerveModules.BACKLEFT.getTurnMotorID(),
    SwerveModules.BACKLEFT.getEncoderPort(),
    SwerveModules.BACKLEFT.getModuleLocation(),
    SwerveModules.BACKLEFT.getEncoderOffset()
  );

  SwerveModule backRightModule = new SwerveModule(
    "BackRight",
    SwerveModules.BACKRIGHT.getDriveMotorID(),
    SwerveModules.BACKRIGHT.getTurnMotorID(),
    SwerveModules.BACKRIGHT.getEncoderPort(),
    SwerveModules.BACKRIGHT.getModuleLocation(),
    SwerveModules.BACKRIGHT.getEncoderOffset()
  );

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle2d(),  new SwerveModulePosition[] {
    frontLeftModule.getPosition(),
    frontRightModule.getPosition(),
    backLeftModule.getPosition(),
    backRightModule.getPosition()
  });

  //dont know why we did this twice but its late and i dont feel like optimizing anymore today
  public Pose2d getLocation() {
    return this.location;
  }

  public Pose2d getPosition() {
    return this.location;
  }

  //returns distance in inches
  public double getDistance() {
    return frontLeftModule.getDistance();
  }

  public AHRS getNavX() {
    return navX;
  }

  public Rotation2d getAngle2d() {
    return new Rotation2d(-navX.getAngle() + 180);
  }

  //in degrees
  public double getAngle() {
    return (-navX.getAngle() + 180) % (360.0);
  }

   //in degrees
   public double getRawAngle() {
    return (-navX.getAngle() + 180);
  }

  //original code:
  /*
   * units::degree_t DriveSubsystem::GetAngleWithOffset(double offset) const 
{
  return units::degree_t(-m_navX->GetAngle() + 180 + offset, 360.0);
}
   */
  //TODO: don't remember if the second parameter of degree_t is stepping distance or wrap, so 50/50 i ported this right
  //the + 360 may or may not be correct
  public double getAngleWithOffset(double offset) {
    return (navX.getAngle() + 180 + offset + 360) % 360.0;
  }

  public DriveSubsystem() {
    SmartDashboard.putBoolean("Enable Drive", true);
    SmartDashboard.putNumber(frontLeftModule.name + " Offset", SwerveModules.FRONTLEFT.getEncoderOffset());
    SmartDashboard.putNumber(frontRightModule.name + " Offset", SwerveModules.FRONTRIGHT.getEncoderOffset());
    SmartDashboard.putNumber(backLeftModule.name + " Offset", SwerveModules.BACKLEFT.getEncoderOffset());
    SmartDashboard.putNumber(backRightModule.name + " Offset", SwerveModules.BACKRIGHT.getEncoderOffset());
    SmartDashboard.putBoolean("Enable Debug", false);
    SmartDashboard.putBoolean("Update Offsets", false);
    turnController = new PIDController(kP, kI, kD);
    turnController.setIntegratorRange(-6.283, 6.283);

    SmartDashboard.putData("Rot", turnController);
    // SmartDashboard.putData("Field Test", field);
    resetGyroscope();
  }

  public void resetGyroscope() {
    navX.reset();

    resetOdometry(new Pose2d());
  }

  public void resetOdometry(Pose2d startPos) {
    odometry.resetPosition(getAngle2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(), 
        frontRightModule.getPosition(), 
        backLeftModule.getPosition(), 
        backRightModule.getPosition()
      }, startPos);
  }

  @Override
  public void periodic() {
    ticks++;
    if(ticks > 5) {
      updateDashboard();
      ticks = 0;
    }

    if(driveEnabled) { advanceSubsystem(); }
  }

  public void updateDashboard() {
    driveEnabled = SmartDashboard.getBoolean("Enable Drive", true);
    if(SmartDashboard.getBoolean("Enable Debug", false)) {
      SmartDashboard.putNumber("Robot Pitch", navX.getPitch());
      SmartDashboard.putNumber("Robot Roll", navX.getRoll());
      SmartDashboard.putNumber(frontLeftModule.name + " Encoder Output", frontLeftModule.getRawAngle());
      SmartDashboard.putNumber(frontRightModule.name + " Encoder Output", frontRightModule.getRawAngle());
      SmartDashboard.putNumber(backLeftModule.name + " Encoder Output", backLeftModule.getRawAngle());
      SmartDashboard.putNumber(backRightModule.name + " Encoder Output", backRightModule.getRawAngle());
    }
    if(SmartDashboard.getBoolean("Update Offsets", false)) {
      SmartDashboard.putBoolean("Update Offsets", false);
      SmartDashboard.putNumber(frontLeftModule.name + " Offset", frontLeftModule.getRawAngle());
      SmartDashboard.putNumber(frontRightModule.name + " Offset", frontRightModule.getRawAngle());
      SmartDashboard.putNumber(backLeftModule.name + " Offset", backLeftModule.getRawAngle());
      SmartDashboard.putNumber(backRightModule.name + " Offset", backRightModule.getRawAngle());
    }
    SmartDashboard.putNumber("IMU Angle", getAngle());
    frontLeftModule.updateAnalogOffset(
        SmartDashboard.getNumber(frontLeftModule.name + " Offset", SwerveModules.FRONTLEFT.getEncoderOffset())
      );
    frontRightModule.updateAnalogOffset(
       SmartDashboard.getNumber(frontRightModule.name + " Offset", SwerveModules.FRONTRIGHT.getEncoderOffset())
     );
    backLeftModule.updateAnalogOffset(
        SmartDashboard.getNumber(backLeftModule.name + " Offset", SwerveModules.BACKLEFT.getEncoderOffset())
      );
    backRightModule.updateAnalogOffset(
        SmartDashboard.getNumber(backRightModule.name + " Offset", SwerveModules.BACKRIGHT.getEncoderOffset())
      );
  }

  public void realignWheels() {
    frontLeftModule.realignWheel();
    frontRightModule.realignWheel();
    backLeftModule.realignWheel();
    backRightModule.realignWheel();
  }

  public void advanceSubsystem() {
    frontLeftModule.readHardware();
    frontRightModule.readHardware();
    backLeftModule.readHardware();
    backRightModule.readHardware();

    location = odometry.update(getAngle2d(), new SwerveModulePosition[] {
      frontLeftModule.getPosition(), 
      frontRightModule.getPosition(), 
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    });

    // field.setRobotPose(location);

    //location = odometry.getPose();

    frontLeftModule.moveTowardsTarget();
    frontRightModule.moveTowardsTarget();;
    backLeftModule.moveTowardsTarget();
    backRightModule.moveTowardsTarget();
  }

  public void drive(double fwd, double str, double rot, boolean fieldRelative, boolean safe) {
    Translation2d centerofRotation = new Translation2d();
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, Rotation2d.fromDegrees(getAngleWithOffset(0))), centerofRotation);
    
    if (!fieldRelative) {
      states = kinematics.toSwerveModuleStates(
        new ChassisSpeeds(fwd, str, rot), centerofRotation);
      //to-do: Wheel desaturation move out of field relative if
      //m_kinematics.DesaturateWheelSpeeds(&states, 1_mps);
    }

    frontLeftModule.setTargetState(states[0]);
    frontRightModule.setTargetState(states[1]);
    backLeftModule.setTargetState(states[2]);
    backRightModule.setTargetState(states[3]);
  }

  public void driveAndAngle(double fwd, double str, double angleDegree, boolean safe) {
    double rot = clamp(turnController.calculate(getRawAngle(), angleDegree), -0.8, 0.8);
    //why tf we adding 0.025 here?
    //whoever did that is retarded as shit
    //wait it was me
    rot = rot + Math.copySign(0.025, rot);
    drive(fwd, str, rot * -1, true, safe);
  }

  public double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  @Override
  public void simulationPeriodic() {}
}
