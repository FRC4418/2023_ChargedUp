// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manndibleCommands;

import frc.robot.subsystems.Arms;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Calibrate extends CommandBase {
  DigitalInput thing = new DigitalInput(9);
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arms m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Calibrate(Arms subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled. (this requires tuning, this numebr represents the position, not speed)
  @Override
  public void execute() {
    System.out.println(thing.get());
    if(thing.get()){
      m_subsystem.grab(0.2);
    }else{
      m_subsystem.grab(0);
      m_subsystem.resetEncoder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.grab(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
