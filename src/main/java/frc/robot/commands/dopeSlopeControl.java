// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DopeSlope;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

public class dopeSlopeControl extends CommandBase {
  /** Creates a new dopeSlopeControl. */
  private final DopeSlope dopeslope;
  private final AutoGamepad autogamepad;

  public dopeSlopeControl(DopeSlope dopeslope, AutoGamepad autogamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dopeslope=dopeslope;
    addRequirements(dopeslope);
    this.autogamepad=autogamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dopeslope.FunnyMethod(autogamepad.getRightY());
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
