// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms.InnerArm;
import frc.robot.subsystems.Arms.OuterArm;

public class TeleopArm extends CommandBase {

  private DoubleSupplier m_InnerOut, m_UpperOut;

  private InnerArm m_innerArm;
  private OuterArm m_outerArm;

  /** Creates a new TeleopArm. */
  public TeleopArm(InnerArm innerArm, OuterArm outerArm, DoubleSupplier upperOut, DoubleSupplier innerOut) {
    m_innerArm = innerArm;
    m_outerArm = outerArm;
    m_InnerOut = innerOut;
    m_UpperOut = upperOut;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(((Math.abs(m_InnerOut.getAsDouble())>0.2) || (Math.abs(m_UpperOut.getAsDouble())>0.2))){
      m_innerArm.raiseWithInput(m_UpperOut.getAsDouble()*-0.3);
      m_outerArm.raiseWithInput(m_InnerOut.getAsDouble()*-0.3);
      // "Borrowed" from another team, not sure of purpose
      // m_arm.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}