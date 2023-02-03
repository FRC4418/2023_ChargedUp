// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Traj extends SubsystemBase {
  private DriveSubsystem driveTrain;
  private Vision vision;
  public Trajectory photonTrak;
  /** Creates a new Trajectory. */
  public Traj(DriveSubsystem driveTrain, Vision vision) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    photonTrak = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0,0.0, new Rotation2d()), List.of(), vision.getCameraToTarget(), driveTrain.getTrajConfig());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    photonTrak = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), List.of(), vision.getCameraToTarget(), driveTrain.getTrajConfig());
    SmartDashboard.putNumber("Hashcode of traj", photonTrak.hashCode());
  }
}
