// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class PathTrajectoryGenerator {

    public static SwerveDrive swerve = RobotContainer.Drive;
    public static PathTrajectoryGenerator Instance = new PathTrajectoryGenerator();

    public static HashMap<String, Command> eventMap;
    public static SwerveAutoBuilder autoBuilder;


}
