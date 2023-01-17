// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math.*;

public class tippedBackwards extends CommandBase {
    private DriveSubsystem driveTrain;
    private float currentPitch;
    private double pitchThreshhold = 0.6;
    private double c;
    private boolean isFinished;
    private int z=100;
  /** Creates a new tippedBackwards. */
  public tippedBackwards(DriveSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPitch = Robot.ahrs.getPitch();

    if(Math.abs(currentPitch) > pitchThreshhold){
        c=(-currentPitch/62);
        driveTrain.setVelocity(c);
    } else if(currentPitch > pitchThreshhold && currentPitch < 0.9){
        c = -c;
        driveTrain.setVelocity(c);
    }
    
    else {
        if(z==100){
            driveTrain.stop();
            z=0;
        }
        else{
            z+=1;
        }
        isFinished = true;
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
