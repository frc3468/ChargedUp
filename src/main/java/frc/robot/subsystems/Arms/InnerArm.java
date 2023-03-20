// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InnerArmConstants;



public class InnerArm extends SubsystemBase {

  private CANSparkMax m_innerMotor;
  private SparkMaxPIDController m_outerPIDController;
  private SparkMaxAnalogSensor m_potentiometor;
  private double m_setPoint;
  public SparkMaxLimitSwitch m_breakstopper;
  /** Creates a new OuterArm. */
  public InnerArm() {
    m_innerMotor = new CANSparkMax(InnerArmConstants.innermotor, MotorType.kBrushed);
    m_innerMotor.setInverted(true);
    m_outerPIDController = m_innerMotor.getPIDController();
    m_potentiometor = m_innerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    m_potentiometor.setInverted(InnerArmConstants.kAnalogSensorInverted);

    m_outerPIDController.setP(InnerArmConstants.innerP);
    m_outerPIDController.setI(InnerArmConstants.innerI);
    m_outerPIDController.setD(InnerArmConstants.innerD);
    m_outerPIDController.setIZone(InnerArmConstants.innerIZone);
    m_outerPIDController.setFF(InnerArmConstants.innerFF);
    m_outerPIDController.setOutputRange(InnerArmConstants.innerMin, InnerArmConstants.innerMax);
    m_outerPIDController.setFeedbackDevice(m_potentiometor);
    m_breakstopper = m_innerMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_breakstopper.enableLimitSwitch(true);
    m_innerMotor.burnFlash();


  }
  public void raise(){
    m_innerMotor.set(InnerArmConstants.raiseSpeed);
  }
  public void raiseETier() {
    m_outerPIDController.setReference(InnerArmConstants.upPIDReferenceE, CANSparkMax.ControlType.kPosition);
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
  public void raiseTravel() {
    m_outerPIDController.setReference(InnerArmConstants.upPIDReferenceT, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.upPIDReferenceT;
  }
  public void lower(){
    m_innerMotor.set(InnerArmConstants.lowerSpeed);
  }
  public void lowerPID() {
    m_outerPIDController.setReference(InnerArmConstants.downPIDReference, CANSparkMax.ControlType.kPosition);
    m_setPoint = InnerArmConstants.downPIDReference; 
  }
  public void stop(){
    m_innerMotor.set(InnerArmConstants.stopSpeed);
  }
  public boolean isAtSetPoint() {
    return (Math.abs(m_setPoint - m_potentiometor.getPosition()) <= InnerArmConstants.innerPIDTolorence);
  }
  public boolean isAtStowed() {
    return (Math.abs(InnerArmConstants.downPIDReference - m_potentiometor.getPosition()) <= InnerArmConstants.innerPIDTolorence);
  }
  public void raiseWithInput(double speed) {
    System.out.println("Inner arm raised at speed: " + speed);
    m_innerMotor.set(speed);
  }
  public void reset(){
    

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("InnerArm", m_potentiometor.getPosition());
    // This method will be called once per scheduler run
  }
}
