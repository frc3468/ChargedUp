// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.camera;

public class UpdateOdom extends CommandBase {
  camera uCamera;
  SwerveDrive uSwerveDrive;
  Boolean finished = false;
  /** Creates a new UpdateOdom. */
  public UpdateOdom(SwerveDrive mSwerveDrive, camera mCamera) {
    uCamera = mCamera;
    uSwerveDrive = mSwerveDrive;
    addRequirements(uCamera);
    addRequirements(uSwerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // converts odometry position to pose 3d
    Pose3d lastpose = new Pose3d(uSwerveDrive.getPose());
    //returns something, unsure if null
    Optional<EstimatedRobotPose> newposestep1 = uCamera.poseestimate(lastpose);
    if( newposestep1 != null){
      EstimatedRobotPose newposestep2 = newposestep1.get();
      Pose3d newposestep3 = newposestep2.estimatedPose;
      uSwerveDrive.resetOdometry(newposestep3.toPose2d());
    }
    finished = true;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
