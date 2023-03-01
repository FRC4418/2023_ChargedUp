// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class armUp extends CommandBase {
  /** Creates a new armUp. */
  private ArmSubsystem arm;
  private double pos;
  public armUp(ArmSubsystem arm, double armPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm= arm;
    this.pos = armPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //dumbArm.inverseSet();
    arm.setPosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
