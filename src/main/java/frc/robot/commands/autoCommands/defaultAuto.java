// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.dopeSlopeCommands.armUp;
import frc.robot.commands.manndibleCommands.ArmsCloseCone;
import frc.robot.commands.manndibleCommands.IntakePull;
import frc.robot.commands.manndibleCommands.IntakePush;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;

public class defaultAuto extends CommandBase {
  /** Creates a new defaultAuto. */
  private ArmSubsystem arm;
  private Intake intake;
  private Arms mandibleArms;
  public defaultAuto(ArmSubsystem arm, Intake intake, Arms mandibleArms) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, mandibleArms);
    this.arm = arm;
    this.intake = intake;
    this.mandibleArms = mandibleArms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
      new ArmsCloseCone(mandibleArms, 0), 
      new IntakePull(intake),
      new WaitCommand(0.5),
      new armUp(arm, Constants.armPositionControl.highPosition),
      new IntakePush(intake)
    ); 
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
