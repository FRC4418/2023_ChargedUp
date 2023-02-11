// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DopeSlope;

public class dopeSlopeExtend extends CommandBase {
  /** Creates a new dopeSlopeControl. */
  private final DopeSlope m_subsystem;
  double position;
  Gamepad l;

  public dopeSlopeExtend(DopeSlope subsystem, double pos, Gamepad e) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = pos;

    this.l = e;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(m_subsystem.getEncoderLength());
    // if(m_subsystem.getEncoderLength()<-position){
    //   m_subsystem.move(.15);
    // }else{
    //   m_subsystem.move(0);
    // }
    m_subsystem.move(.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
