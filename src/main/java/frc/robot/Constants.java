// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
    public static final int kOperatorLeftJoystickPort = 1;
    public static final int kOperatorRightJoystickPort = 2;

  }

  public static class PneumaticsConstants {
    public static final int kModuleID = 31;
    public static final PneumaticsModuleType kModuleType = PneumaticsModuleType.REVPH;
  }

  public static class RearArmConstants {
    public static final int kSparkMaxID = 13;
    public static final CANSparkMaxLowLevel.MotorType kMotorType = CANSparkMaxLowLevel.MotorType.kBrushless;
    public static final CANSparkMax.IdleMode kIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final boolean kMotorInverted = false;

    public static final SparkMaxAnalogSensor.Mode kAnalogSensorMode = SparkMaxAnalogSensor.Mode.kAbsolute;
    public static final boolean kAnalogSensorInverted = true;
    public static final double kAnalogSensorPositionConversionFactor = 1.0;
    
    public static final CANSparkMax.ControlType kPIDControlType = CANSparkMax.ControlType.kPosition;
    public static final double kPIDMinOutput = -1.0;
    public static final double kPIDMaxOutput = 1.0;
    public static final double kPIDTolerance = 0.025;

    public static final double kPID_P = 1.0;
    public static final double kPID_I = 0.0;
    public static final double kPID_D = 0.0;
    public static final double kPID_FF = 0.0;
    public static final double kPID_IZone = 0.0;
    public static final double kPID_DFilter = 0.0;

    public static final double kManualForwardSpeed = 1.0;
    public static final double kManualReverseSpeed = -1.0;
    public static final double kManualScaleFactor = 1.0;

    public static final double kStowSetpoint = 0.0;
    public static final double kMoveSetpoint = 0.0;
    public static final double kLowSetpoint = 0.0;
    public static final double kMidSetpoint = 0.0;
    public static final double kHighSetpoint = 0.0;
    public static final double kLoadSetpoint = 0.0;
  }


  public static class ForeArmConstants {
    public static final int kSparkMaxID = 14;
    public static final CANSparkMaxLowLevel.MotorType kMotorType = CANSparkMaxLowLevel.MotorType.kBrushless;
    public static final CANSparkMax.IdleMode kIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final boolean kMotorInverted = false;

    public static final SparkMaxAnalogSensor.Mode kAnalogSensorMode = SparkMaxAnalogSensor.Mode.kAbsolute;
    public static final boolean kAnalogSensorInverted = true;
    public static final double kAnalogSensorPositionConversionFactor = 1.0;
    
    public static final CANSparkMax.ControlType kPIDControlType = CANSparkMax.ControlType.kPosition;
    public static final double kPIDMinOutput = -1.0;
    public static final double kPIDMaxOutput = 1.0;
    public static final double kPIDTolerance = 0.025;

    public static final double kPID_P = 1.0;
    public static final double kPID_I = 0.0;
    public static final double kPID_D = 0.0;
    public static final double kPID_FF = 0.0;
    public static final double kPID_IZone = 0.0;
    public static final double kPID_DFilter = 0.0;

    public static final double kManualForwardSpeed = 0.3;
    public static final double kManualReverseSpeed = -0.3;
    public static final double kManualScaleFactor = 0.5;

    public static final double kStowSetpoint = 0.0;
    public static final double kMoveSetpoint = 0.0;
    public static final double kLowSetpoint = 0.0;
    public static final double kMidSetpoint = 0.0;
    public static final double kHighSetpoint = 0.0;
    public static final double kLoadSetpoint = 0.0;
  }


  public static class ClawConstants {
    public static final int kOpenPistonID = 0;
    public static final int kClosePistonID = 1;
  }
}
