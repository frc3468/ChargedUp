// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.config.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Clawconstants {
    public static final int ModuleID = 31;
    public static final int OpenClaw = 0;
    public static final int CloseClaw = 1;
    public static final int lazersensor = 2;
    public static final int overideClawOpen = 1;
    public static final int overideClawClose = 1;


  }
  
  
  public static class CameraConstants{
    // keeps distance from the camera to the robot in meter. 
    //Metric forever
    public static Transform3d camToRobot = new Transform3d(new Translation3d(0, 0,0), new Rotation3d(0, 0, 0));
  }
//Joystick deadband.  Filters out stick drift
  public static final class Swerve {
    public static final double stickDeadband = 0.1;
    /* GYRO Constants */
    // Declare Gyro location on CANbus
    public static final int gyroid = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    // TODO not sure we want to zero the gyro out the gate (upon init)
    //public static final int gyro = 0;

    /* SWERVE DRIVETRAIN CONSTANTS */
    /** WHEELBASE
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double trackWidth = 0.6254; //Units.inchesToMeters(24.625);
    /** 
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double wheelBase = 0.6096; //units.inchesToMeters(24.625)

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (21.42857 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = 
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    /*
    * KP = Propotional Gain constant of the PIDF controller
    * KI = Integral Gain constant of the PIDF controller
    * KD = Derivitive Gain constant of the PIDF controller
    * KFF = Feed-forward gain of the PIDF controller
    * Use SPARKMAX GUI to tune and save paremeters
    */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
 
  
    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // 4.5 meters per second
    public static final double maxAngularVelocity = 11.5;//11.5
    // var speeds = new chassisspeeds(3.0,-2.0, Math.pi);
    //TODO turtle mode
    public static final double turtleSpeed = 1.35; // 1.35 meters per second

    /*  Controller offset.  Alters the max output of joysticks to throttle X,Y,Turn */
    // TODO not called yet
    public static final double j_maxXController = .5;//forward/rev
    public static final double j_maxYController = .5; // strafe left/right
    public static final double j_maxTurnController = .8;// turn (right Joystick)

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
        /*Encoder pos 71.367 degrees */

    public static final class FrontLeftSwerveMod {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(251.104);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
        /*Encoder pos 105.469 degrees */
    public static final class FrontRightSwerveMod {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.148);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
        /* Encoder 288.105 degrees */
    public static final class RearLeftSwerveMod {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(339.082);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
        /* Endoder 338.730 degrees */
    public static final class RearRightSwerveMod {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(243.281);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
      /* END SWERVE DRIVETRAIN CONSTANTS */
}
public static final class OuterArmConstants {
    public static final int outermotor = 13;
    public static final int overrideDown = 2; // button channel? 
    public static final int overrideUp = 3; // button channel? 
    public static final double raiseSpeed = 1.0;
    public static final double lowerSpeed = -0.6;
    public static final double stopSpeed = 0.0;
    public static final double outerP = 10.0;
    public static final double outerI = 0.0;
    public static final double outerD = 0.0;
    public static final double outerIZone = 0.0;
    public static final double outerFF = 0.0;
    public static final double outerMin = -1.0;
    public static final double outerMax = 1.0;
    public static final double upPIDReferenceE = 0.80;
    public static final double upPIDReferenceM = 1.26;
    public static final double upPIDReferenceS = 1.51;
    public static final double downPIDReference = 0.53;
    public static final double upPIDReferenceT = 1.2;
    public static final double startingConfig = 2.5;
    public static final double outerPIDTolorence = 0.04;
    public static final boolean kAnalogSensorInverted = true;
  // outer arm
  /*
   *driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
   * 
   */

}

public static final class InnerArmConstants {
  public static final int innermotor = 14;
  public static final int overrideDown = 2; // button channel? 
  public static final int overrideUp = 3; // button channel? 
  public static final int raiseSpeed = 1;
  public static final int lowerSpeed = -1;
  public static final int stopSpeed = 0;
  public static final double innerP = 10.0;
  public static final double innerI = 0.0;
  public static final double innerD = 0.0;
  public static final double innerIZone = 0.0;
  public static final double innerFF = 0.0;
  public static final double innerMin = -1.0;
  public static final double innerMax = 1.0;
  public static final double upPIDReferenceE = 1.26; //A
  public static final double upPIDReferenceM = 1.28; //B
  public static final double upPIDReferenceS = 1.43; //Y
  public static final double downPIDReference = 1.25;  //X
  public static final double upPIDReferenceT = 2.27;  //X
  public static final double innerPIDTolorence = 0.03;
  public static final boolean kAnalogSensorInverted = false;
//inner arm
}
}
