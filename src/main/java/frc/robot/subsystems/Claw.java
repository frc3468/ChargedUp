// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private DoubleSolenoid m_solenoid;

  /** Creates a new Claw. */
  public Claw() {
    m_solenoid = new DoubleSolenoid(PneumaticsConstants.kModuleID, PneumaticsConstants.kModuleType, ClawConstants.kOpenPistonID, ClawConstants.kClosePistonID);
  }

  public void open() {
    m_solenoid.set(Value.kForward);
  }

  public void close() {
    m_solenoid.set(Value.kReverse);
  }

  public boolean isClosed() {
    return m_solenoid.get() == Value.kReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase openCommand() {
    return runOnce(() -> open());
  }

  public CommandBase closeCommand() {
    return runOnce(() -> close());
  }
}
