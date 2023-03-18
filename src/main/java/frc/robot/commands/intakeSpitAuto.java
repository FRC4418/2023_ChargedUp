// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.util.plot.Playground.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rollers;

public class intakeSpitAuto extends CommandBase {
  /** Creates a new intakeSuck. */
  Rollers rollers = new Rollers();
  Timer timer = new Timer();

  public intakeSpitAuto(Rollers rollers) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollers);
    this.rollers = rollers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollers.intakeSpin(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.2){
      return true;
    } else {
      return false;
    }
  }
}
