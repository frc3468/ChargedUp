// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuterArmConstants;



public class OuterArm extends SubsystemBase {

  private CANSparkMax m_outerMotor;
  private SparkMaxPIDController m_outerPIDController;
  private SparkMaxAbsoluteEncoder m_potentiometor;
  private double m_setPoint;
  public SparkMaxLimitSwitch m_breakstopper;

  /** Creates a new OuterArm. */
  public OuterArm() {
    m_outerMotor = new CANSparkMax(OuterArmConstants.outermotor, MotorType.kBrushless);
    m_outerMotor.setInverted(true);
    m_outerPIDController = m_outerMotor.getPIDController();
    m_potentiometor = m_outerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_potentiometor.setInverted(OuterArmConstants.kAnalogSensorInverted);

    m_outerPIDController.setP(OuterArmConstants.outerP);
    m_outerPIDController.setI(OuterArmConstants.outerI);
    m_outerPIDController.setD(OuterArmConstants.outerD);
    m_outerPIDController.setIZone(OuterArmConstants.outerIZone);
    m_outerPIDController.setFF(OuterArmConstants.outerFF);
    m_outerPIDController.setOutputRange(OuterArmConstants.outerMin, OuterArmConstants.outerMax);
    m_outerPIDController.setFeedbackDevice(m_potentiometor);
    m_breakstopper = m_outerMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_breakstopper.enableLimitSwitch(true);
    m_outerMotor.burnFlash();
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
  public void raiseTravel() {
    m_outerPIDController.setReference(OuterArmConstants.upPIDReferenceT, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.upPIDReferenceT;
  }
  public void raiseHuman() {
    m_outerPIDController.setReference(OuterArmConstants.upPIDReferenceH, CANSparkMax.ControlType.kPosition);
    m_setPoint = OuterArmConstants.upPIDReferenceT;
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
  public boolean isAtStowed() {
    return (Math.abs(OuterArmConstants.downPIDReference - m_potentiometor.getPosition()) <= OuterArmConstants.outerPIDTolorence);
  }
  

  public void raiseWithInput(double speed) {
    if(speed != 0) {
     System.out.println("Outer arm raised at speed: " + speed);
    }
    m_outerMotor.set(speed);}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("outer arm", m_potentiometor.getPosition());
    // This method will be called once per scheduler run
  }
}
