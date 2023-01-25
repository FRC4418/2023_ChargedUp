// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.PhotonVisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class rangedDrive extends CommandBase {
  private DriveSubsystem driveTrain;
  private double distanceThreshhold = 1.8;
  private double currentDistance;
  private double speed;
  public static PhotonCamera camera = new PhotonCamera("USB_Camera");

  /** Creates a new rangedDrive. */
  public rangedDrive(DriveSubsystem driveTrain) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SetPipeline(PhotonVisionConstants.aprilTagPipeline);

    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {

      final PhotonTrackedTarget target = result.getBestTarget();
      currentDistance = PhotonUtils.calculateDistanceToTargetMeters(0.65, 1.0, Units.degreesToRadians(-7.3), Units.degreesToRadians(target.getPitch()));
      speed = 0.1;

    }

    if (currentDistance > distanceThreshhold) {

      driveTrain.setVelocity(speed);

    } 
    else {

      driveTrain.stop();

    }

  }

  public void SetPipeline(int pipeline) {

    camera.setPipelineIndex(pipeline);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
