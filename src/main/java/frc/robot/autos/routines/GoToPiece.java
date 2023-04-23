// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import org.apache.commons.collections4.sequence.InsertCommand;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.PathTrajectoryGenerator;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPiece extends SequentialCommandGroup {

  SwerveAutoBuilder autoBuilder = PathTrajectoryGenerator.getAutoBuilder();

  Command getPiece = autoBuilder.fullAuto(PathTrajectoryGenerator.gotopiece());
  /** Creates a new GoToPiece. */
  public GoToPiece(SwerveDrive Swerve) {

    addCommands(
    new InstantCommand(() -> Swerve.resetOdometry(new Pose2d())),
    new InstantCommand(() -> Swerve.zeroGyro()),
    getPiece,
    new InstantCommand(() -> Swerve.zeroGyro180()));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
