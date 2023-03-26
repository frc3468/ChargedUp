// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Setpoints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms.InnerArm;

public class InnerArmRaiseH extends CommandBase {
  /** Creates a new InnerArmRaiseH. */
  private InnerArm m_subsystem;
  public InnerArmRaiseH(InnerArm subsystem) {
   m_subsystem = subsystem;
   addRequirements(m_subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("RaiseDone", false);
    m_subsystem.raiseHuman();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("RaiseDone", true);
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isAtSetPoint() || m_subsystem.m_breakstopper.isPressed();
  }
}
