// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.InnerArmRaiseM;
import frc.robot.commands.InnerArmStowed;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.OuterArmRaiseM;
import frc.robot.commands.OuterArmStowed;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.*;
import frc.robot.subsystems.Arms.InnerArm;
import frc.robot.subsystems.Arms.OuterArm;
import frc.robot.subsystems.Claw;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAuto extends SequentialCommandGroup {
  /** Creates a new SideAuto. */
  private final SwerveDrive s_Swerve;
  private final Claw m_Claw;
  private final InnerArm m_InnerArm;
  private final OuterArm m_OuterArm; 


  public SideAuto(SwerveDrive p_swerve, Claw p_claw, InnerArm p_innerarm, OuterArm p_outerarm ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    s_Swerve = p_swerve;
    m_Claw = p_claw ;
    m_InnerArm = p_innerarm;
    m_OuterArm = p_outerarm;
    addCommands(
    new InstantCommand(() -> s_Swerve.zeroGyro()),
    new CloseClaw(m_Claw),  
    new InstantCommand(() -> s_Swerve.turtleMode()),
    new TeleopSwerve(s_Swerve, () -> -3, () -> 0.0, () -> 0.0 , () -> false).withTimeout(.2),
    // because it's allready in a scg we don't need to make a new one
    new InnerArmRaiseM(m_InnerArm),
    new OuterArmRaiseM(m_OuterArm),
    new TeleopSwerve(s_Swerve, () -> 3, () -> 0.0, () -> 0.0 , () -> false).withTimeout(1),
    new OpenClaw(m_Claw),
    new TeleopSwerve(s_Swerve,() -> -5.0,() -> 0.0,() ->0.0, () -> false).withTimeout(0.55),
    new SequentialCommandGroup(
   // as opposed to here where we do.
      new WaitCommand(0.2),
      new InnerArmStowed(m_InnerArm),
      new OuterArmStowed(m_OuterArm)
    ),
    new InstantCommand(() -> s_Swerve.turtleMode()),
    new TeleopSwerve(s_Swerve,() -> -5.0,() -> 0.0,() ->0.0, () -> false).withTimeout(1.00),
    new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> .5, () -> false).withTimeout(0.5),
    new InstantCommand(() -> s_Swerve.zeroGyro()), // 180 the Gyro
    new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> .25, () -> false).withTimeout(0.25)
    ); // TODO place holder for now, replace once we have auto modes
  }
    ;
  }

