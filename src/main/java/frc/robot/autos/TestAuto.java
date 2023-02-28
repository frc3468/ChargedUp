// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.UpdateOdom;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto(SwerveDrive m_SwerveDrive, Camera m_Camera) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new UpdateOdom(m_SwerveDrive, m_Camera).withTimeout(10),
      new InstantCommand(() -> m_SwerveDrive.zeroGyro(m_SwerveDrive.getPose().getRotation().getRadians())),
      new DriveToPoint(m_SwerveDrive, -0.04, 1.02)
    );
  }
}
