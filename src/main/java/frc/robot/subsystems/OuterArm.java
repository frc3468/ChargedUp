// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuterArmConstants;



public class OuterArm extends SubsystemBase {

  private CANSparkMax m_outerMotor;
  private SparkMaxPIDController m_outerPIDController;
  private SparkMaxAnalogSensor m_potentiometor;
  private double m_setPoint;

  /** Creates a new OuterArm. */
  public void OuterArm() {
    m_outerMotor = new CANSparkMax(OuterArmConstants.outermotor, MotorType.kBrushless);
     
    m_outerPIDController = m_outerMotor.getPIDController();
    m_potentiometor = m_outerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

    m_outerPIDController.setP(OuterArmConstants.outerP);
    m_outerPIDController.setI(OuterArmConstants.outerI);
    m_outerPIDController.setD(OuterArmConstants.outerD);
    m_outerPIDController.setIZone(OuterArmConstants.outerIZone);
    m_outerPIDController.setFF(OuterArmConstants.outerFF);
    m_outerPIDController.setOutputRange(OuterArmConstants.outerMin, OuterArmConstants.outerMax);
    m_outerPIDController.setFeedbackDevice(m_potentiometor);

  }
  public void raise(){
    m_outerMotor.set(OuterArmConstants.raiseSpeed);
  }
  public void raiseETier() {
    m_outerPIDController.setReference(OuterArmConstants.upPIDReferenceE, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.upPIDReferenceE;
  }
  public void raiseMid() {
    m_outerPIDController.setReference(OuterArmConstants.upPIDReferenceM, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.upPIDReferenceM;
  }
  public void raiseSTier() {
    m_outerPIDController.setReference(OuterArmConstants.upPIDReferenceS, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.upPIDReferenceS;
  }
  public void lower(){
    m_outerMotor.set(OuterArmConstants.lowerSpeed);
  }
  public void lowerPID() {
    m_outerPIDController.setReference(OuterArmConstants.downPIDReference, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.downPIDReference; 
  }
  public void stop(){
    m_outerMotor.set(OuterArmConstants.stopSpeed);
  }
  public boolean isAtSetPoint() {
    return (Math.abs(m_setPoint - m_potentiometor.getPosition()) <= OuterArmConstants.outerPIDTolorence);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
