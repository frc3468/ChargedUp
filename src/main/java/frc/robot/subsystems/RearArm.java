// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RearArmConstants;

public class RearArm extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxAnalogSensor m_analogEncoder;
  private SparkMaxPIDController m_pidController;
  private double m_setpoint;

  /** Creates a new RearArm. */
  public RearArm() {
    m_motor = new CANSparkMax(RearArmConstants.kSparkMaxID, RearArmConstants.kMotorType);
    m_motor.setIdleMode(RearArmConstants.kIdleMode);
    m_motor.setInverted(RearArmConstants.kMotorInverted);

    m_analogEncoder = m_motor.getAnalog(RearArmConstants.kAnalogSensorMode);
    m_analogEncoder.setInverted(RearArmConstants.kAnalogSensorInverted);
    m_analogEncoder.setPositionConversionFactor(RearArmConstants.kAnalogSensorPositionConversionFactor);

    m_pidController = m_motor.getPIDController();
    m_pidController.setOutputRange(RearArmConstants.kPIDMinOutput, RearArmConstants.kPIDMaxOutput);
    m_pidController.setFeedbackDevice(m_analogEncoder);
    
    m_pidController.setP(RearArmConstants.kPID_P);
    m_pidController.setI(RearArmConstants.kPID_I);
    m_pidController.setD(RearArmConstants.kPID_D);
    m_pidController.setFF(RearArmConstants.kPID_FF);
    m_pidController.setIZone(RearArmConstants.kPID_IZone);
    m_pidController.setDFilter(RearArmConstants.kPID_DFilter);

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
    m_pidController.setReference(m_setpoint, RearArmConstants.kPIDControlType);
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_setpoint - m_analogEncoder.getPosition()) < RearArmConstants.kPIDTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase setSpeedCommand(double speed) {
    return runEnd(() -> setSpeed(speed), () -> setSpeed(RearArmConstants.kStopSpeed));
  }

  public CommandBase setPositionCommand(double setpoint) {
    return run(() -> setSetpoint(setpoint)).until(() -> isAtSetpoint());
  }
}
