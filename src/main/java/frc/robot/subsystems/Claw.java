// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Clawconstants;
 


public class Claw extends SubsystemBase {
  private DigitalInput lazerSensor;
  private final Compressor comp = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  
  /** Creates a new Claw. */
  public Claw() {
    lazerSensor = new DigitalInput(Clawconstants.lazersensor);

  }
 public void Open(){
  solenoid.set(Value.kForward);
 }
 public void Close(){
  solenoid.set(Value.kReverse);
 }
 public boolean getLazerSenser() {
  return !lazerSensor.get();
}
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Sensor", !lazerSensor.get());
    // This method will be called once per scheduler run
  }
}
