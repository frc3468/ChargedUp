// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InnerArmConstants;



public class InnerArm extends SubsystemBase {

  private CANSparkMax m_outerMotor;
  private SparkMaxPIDController m_outerPIDController;
  private SparkMaxAnalogSensor m_potentiometor;
  private double m_setPoint;

  /** Creates a new OuterArm. */
  public void InnerArm() {
    m_outerMotor = new CANSparkMax(InnerArmConstants.innermotor, MotorType.kBrushless);
     
    m_outerPIDController = m_outerMotor.getPIDController();
    m_potentiometor = m_outerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

    m_outerPIDController.setP(InnerArmConstants.innerP);
    m_outerPIDController.setI(InnerArmConstants.innerI);
    m_outerPIDController.setD(InnerArmConstants.innerD);
    m_outerPIDController.setIZone(InnerArmConstants.innerIZone);
    m_outerPIDController.setFF(InnerArmConstants.innerFF);
    m_outerPIDController.setOutputRange(InnerArmConstants.innerMin, InnerArmConstants.innerMax);
    m_outerPIDController.setFeedbackDevice(m_potentiometor);

  }
  public void raise(){
    m_outerMotor.set(InnerArmConstants.raiseSpeed);
  }
  public void raiseETier() {
    m_outerPIDController.setReference(InnerArmConstants.upPIDReferenceM, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.upPIDReferenceM;
  }
  public void raiseMid() {
    m_outerPIDController.setReference(InnerArmConstants.upPIDReferenceM, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.upPIDReferenceM;
  }
  public void raiseSTier() {
    m_outerPIDController.setReference(InnerArmConstants.upPIDReferenceS, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.upPIDReferenceS;
  }
  public void lower(){
    m_outerMotor.set(InnerArmConstants.lowerSpeed);
  }
  public void lowerPID() {
    m_outerPIDController.setReference(InnerArmConstants.downPIDReference, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.downPIDReference; 
  }
  public void stop(){
    m_outerMotor.set(InnerArmConstants.stopSpeed);
  }
  public boolean isAtSetPoint() {
    return (Math.abs(m_setPoint - m_potentiometor.getPosition()) <= InnerArmConstants.innerPIDTolorence);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
