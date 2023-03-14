// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForeArmConstants;

public class ForeArm extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxAnalogSensor m_analogEncoder;
  private SparkMaxPIDController m_pidController;
  private double m_setpoint;
  
  /** Creates a new ForeArm. */
  public ForeArm() {
    m_motor = new CANSparkMax(ForeArmConstants.kSparkMaxID, ForeArmConstants.kMotorType);
    m_motor.setIdleMode(ForeArmConstants.kIdleMode);
    m_motor.setInverted(ForeArmConstants.kMotorInverted);

    m_analogEncoder = m_motor.getAnalog(ForeArmConstants.kAnalogSensorMode);
    m_analogEncoder.setInverted(ForeArmConstants.kAnalogSensorInverted);
    m_analogEncoder.setPositionConversionFactor(ForeArmConstants.kAnalogSensorPositionConversionFactor);

    m_pidController = m_motor.getPIDController();
    m_pidController.setOutputRange(ForeArmConstants.kPIDMinOutput, ForeArmConstants.kPIDMaxOutput);
    m_pidController.setFeedbackDevice(m_analogEncoder);
    
    m_pidController.setP(ForeArmConstants.kPID_P);
    m_pidController.setI(ForeArmConstants.kPID_I);
    m_pidController.setD(ForeArmConstants.kPID_D);
    m_pidController.setFF(ForeArmConstants.kPID_FF);
    m_pidController.setIZone(ForeArmConstants.kPID_IZone);
    m_pidController.setDFilter(ForeArmConstants.kPID_DFilter);

    m_motor.burnFlash();
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public double getSpeed() {
    return m_motor.get();
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    m_pidController.setReference(m_setpoint, ForeArmConstants.kPIDControlType);
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_setpoint - m_analogEncoder.getPosition()) < ForeArmConstants.kPIDTolerance;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase setSpeedCommand(DoubleSupplier speed) {
    return runEnd(() -> setSpeed(speed.getAsDouble()), () -> setSpeed(ForeArmConstants.kStopSpeed));
  }

  public CommandBase setPositionCommand(DoubleSupplier setpoint) {
    return run(() -> setSetpoint(setpoint.getAsDouble())).until(() -> isAtSetpoint());
  }
}
