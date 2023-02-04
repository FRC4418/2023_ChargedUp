// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  //create camera object
  //public static PhotonCamera camera = new PhotonCamera("USB_Camera");
  public static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  //create AprilTagFieldLayout object to map tags to locations
  private static AprilTagFieldLayout aprilTagFieldLayout;
  //map camera to the drive train
  public static Transform3d cameraToDrivetrain;
  //create new PhotonVision Pose Estimator
  public static PhotonPoseEstimator m_poseEstimator;
  //lasat frame
  public static PhotonTrackedTarget lastTarget;

  public double timeStamp;
  public double previousTimeStamp;
  public double previousPipelineTimestamp;
  public PhotonPipelineResult previousResult;
  public PhotonPipelineResult result;
  /** Creates a new vision. */
  public Vision() {
    result = camera.getLatestResult();
    previousResult = result;
    //Transform3d tagTogoal = new Transform3d(new Translation3d(1.5,0.0,0.0), new Rotation3d(0.0,0.0,Math.PI));
    previousPipelineTimestamp = 0.0;
    
    //populate transform3d camera realtive to robot
    cameraToDrivetrain = new Transform3d(
      new Translation3d(0.35,0.36,0.14), 
      new Rotation3d(0,0,0)
      );
  }

  public Pose2d getCameraToTarget(){
    result = camera.getLatestResult();

    
    if (result == null) {

      result = previousResult;

    }
    else {

      previousResult = result;

    }

    // double timeStamp = result.getTimestampSeconds();    
    // if(timeStamp != previousPipelineTimestamp && result.hasTargets()){
    //   previousPipelineTimestamp = timeStamp;
    //   previousResult = result;
    // }
    //if(result == null){
      //return null;
    //}
    
    PhotonTrackedTarget target = result.getBestTarget(); 
    if(target == null){
      return null;
    }
    //create transfromToTarget using values from camera
    Transform3d transformToTarget = target.getBestCameraToTarget();   
    if(transformToTarget == null){
      return null;
    }
    //translate 3D pose to 2D pose and returns
    Pose2d resultPose;
    resultPose = new Pose2d(transformToTarget.getX()-0.25, transformToTarget.getY(), transformToTarget.getRotation().toRotation2d());
    return resultPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
