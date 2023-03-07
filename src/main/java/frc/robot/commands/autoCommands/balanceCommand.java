// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class balanceCommand extends CommandBase {
  /** Creates a new balanceCommand. */
  private DriveSubsystem driveTrain;
  private double currentPitch;
  private double pitchThreshhold = 0.6;
  private double velocity;
  private boolean isFinished;
  private int z = 100;

  public balanceCommand(DriveSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPitch = driveTrain.getPitch();

    if (Math.abs(currentPitch) > pitchThreshhold) {          
      velocity = (-(currentPitch) / 62);
      //check if .tankDrive or a regular .set type method should be used
      driveTrain.tankDrive(velocity, velocity);
  } else  if(Math.abs(currentPitch) > pitchThreshhold && Math.abs(currentPitch) < 3d){
      velocity = (-(currentPitch) / 20);
  } else {
    if (z == 100) {
      driveTrain.arcadeDrive(0,0);
      z = 0;
  } 
  else {
      z += 1;
  }
  isFinished = true;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
