// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.miscCommands;

import frc.robot.constants.Ports.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class resetOdometry extends CommandBase {
  private final DriveSubsystem driveTrain;
  private final Arms arms;
  private final ArmSubsystem arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public resetOdometry(DriveSubsystem driveTrain, Arms arms, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, arms, arm);
    this.driveTrain = driveTrain;
    this.arms = arms;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.resetOdometry(new Pose2d());
    arms.resetEncoders();
    arm.resetSensor();
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
