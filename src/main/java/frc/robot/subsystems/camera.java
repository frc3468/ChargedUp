// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class camera extends SubsystemBase {
  /** Creates a new camera. */
  
  PhotonPoseEstimator  poseRefinery;
  PhotonCamera machineCamera;
  String currentfilter;
  boolean didISucceed;
  AprilTagFieldLayout aLayout;
  boolean layoutfailed;
  Pose3d lastpose3d;
  
  

  public camera() {
    layoutfailed = false;
    didISucceed = true; 
    currentfilter = "AprilTag";
    machineCamera = new PhotonCamera("3468"); 
    machineCamera.setDriverMode(!didISucceed);
    changeFilter(currentfilter);

    try {
      aLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      layoutfailed = true;

      
    }
    if(layoutfailed == false){
    poseRefinery = new PhotonPoseEstimator(aLayout,org.photonvision.PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY ,machineCamera,Constants.CameraConstants.camToRobot);
    }
  }

  public void enabledrivermode (){
    machineCamera.setDriverMode(true);
  }
  public void disabledrivermode(){
    machineCamera.setDriverMode(false);
  }

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
    // Comment this out probably but could be useful. Essentially  just displays camera latency 
  }
  public PhotonPipelineResult getResult(){
     PhotonPipelineResult results =  machineCamera.getLatestResult();
     // gives photonPipeline results if any valid targets else returns null
     if(results.hasTargets() == false){
      return null;
     }
     else{
      return results;
     }
  }

  public List<PhotonTrackedTarget> gettargets(){
    // returns a list of photon tracked targets with the corropnding information regarding them
    PhotonPipelineResult raw = getResult();
    if(raw != null)
    {return raw.getTargets();}
    else{ 
      return null;}
    
    // For the uninformed this still requires further refinement to acess said information 
  }
  public PhotonTrackedTarget getBesttarget(){
    PhotonPipelineResult raw = getResult();
    if(raw != null){
      return raw.getBestTarget();
    }
    else {return null;}


    }

  public void changeFilter(String filter){
    filter = filter.toLowerCase();
    if(filter == "cube" || filter ==  "cubes" ){
      machineCamera.setPipelineIndex(2);
    }
    else if(filter == "cone" || filter == "cones"){
      machineCamera.setPipelineIndex(1);
    }
    else if(filter == "tag" || filter == "tags" || filter == "apriltags"){
      machineCamera.setPipelineIndex(0);
    }
  }
  // for the following -1 is an error code 
  public double getbestyaw(){
    if(getBesttarget() != null){
    return getBesttarget().getYaw();}
    else{return -1;}
  }
  public double getbestpitch(){
    if(getBesttarget() != null){return getBesttarget().getPitch();}
    else{return -1;}
    
  }
  public double getbestarea(){
    return getBesttarget().getArea();
  }
  public double getBestSkew(){
    return getBesttarget().getSkew();
  }

  public double getbestaprilid(){
    if(getBesttarget() != null && currentfilter == "tag"){
      return getBesttarget().getFiducialId();
    }
    else{
      return -1;
    }
  }
  //for the following null is an error
  public Transform3d getCamtoTarget(){
    if(getBesttarget() != null){
      return getBesttarget().getBestCameraToTarget();
    }
    else{ return null;}
  }



  //UGHHHH No transform 2d for non apriltag targets. 


 
  public Optional<EstimatedRobotPose> poseestimate(Pose3d lastpose3d){

    if(layoutfailed = false || gettargets() != null){
      poseRefinery.setReferencePose(lastpose3d);
      return poseRefinery.update();
      
    }
    else{
      return null;
    }
  };
}