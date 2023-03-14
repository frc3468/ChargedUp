// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoint extends SequentialCommandGroup {
  /** Creates a new DriveToPoint. */
  public DriveToPoint(SwerveDrive m_Drive, double posx, double posy) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get the intended pose 
    double transformx = Math.round(posx - m_Drive.getPose().getX()*100)/100 ;
    double transformy = Math.round(posy - m_Drive.getPose().getY() *100)/100 ;
    double speed = 0.2;
    System.out.print("got to movement");



    addCommands(
      new TeleopSwerve(m_Drive, () -> transformx, () -> transformy, () -> 0.0, () -> false).until(() -> Math.round(m_Drive.getPose().getX()*100)/100 == posx ||  Math.round(m_Drive.getPose().getY()*100)/100 == posy)
    );
}}
