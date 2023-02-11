// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DopeSlope;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ResetAllEncoders extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arms m_arm;
  private final DopeSlope m_DopeSlope;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ResetAllEncoders(Arms arm, DopeSlope dope) {
    m_arm = arm;
    m_DopeSlope = dope;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    addRequirements(dope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled. (this requires tuning, this numebr represents the position, not speed)
  @Override
  public void execute() {
    m_arm.resetEncoders();
    m_DopeSlope.resetEncoders(); 
    System.out.println("reset encoder things");
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
