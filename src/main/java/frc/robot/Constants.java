// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //too lazy to do enum for drive constants
  // first four constants are from swervedrivespecialties github
  public static class Drive {
    public static final double DISTANCE_PER_ROT = 1.0/8.16;
    public static final double WHEEL_DIAMETER = 4;
    public static final double DRIVE_REDUCTION = 8.33/1.0;
    public static final double DRIVE_CONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_REDUCTION;
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;
    public static final double ROT_POSITION_CONVERSION_FACTOR = 2 * Math.PI / (18.0 / 1.0);
    
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int TURN_CURRENT_LIMIT = 20;
    public static final int SECONDARY_CURRENT_OFFSET = 5;
    
  }

  public static class Turn {
    public static final double P = 1.5;
    public static final double I = 0;
    public static final double D = 0.5;
  }

  public static enum SwerveModules {
    RONTLEFT(2,4,3, new Translation2d(0.381, 0.381), -(0.0431 + 0.39216 + 3.14159265 - (2 * 3.14159265))),
    FRONTRIGHT(3,2,1, new Translation2d(0.381, -0.381), -(1.57568)),
    BACKLEFT(0,8,7, new Translation2d(-0.381, 0.381), -(-0.97338 + 6.2831)),
    BACKRIGHT(1,6,5, new Translation2d(-.381, -0.381), -(0.278867));

     //analog absolute encoder port (analog in on roborio)
     private final int encoderPort;
     //neo drive motor id on can loop
     private final int driveMotorID;
     //neo turn motor id on can loop
     private final int turnMotorID;
     //module location in 2d space
     private final Translation2d moduleLocation;
     //absolute encoder offset in radians
     private final double encoderOffset;
     SwerveModules(int encoderPort, int driveID, int turnID, Translation2d location, double encoderOffset) {
       this.encoderPort = encoderPort;
       this.driveMotorID = driveID;
       this.turnMotorID = turnID;
       this.moduleLocation = location;
       this.encoderOffset = encoderOffset;
     }
 
     public int getEncoderPort() {
       return this.encoderPort;
     }
 
     public int getDriveMotorID() {
       return this.driveMotorID;
     }
 
     public int getTurnMotorID() {
       return this.turnMotorID;
     }
 
     public Translation2d getModuleLocation() {
       return this.moduleLocation;
     } 
 
     public double getEncoderOffset() {
       return this.encoderOffset;
     }
 
   }
 
 }
