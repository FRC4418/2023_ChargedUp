// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manndibleCommands;

import frc.robot.constants.Constants;
import frc.robot.subsystems.Arms;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmsCloseCubeAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arms arms;
  private Timer timer = new Timer();
  double position;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmsCloseCubeAuto(Arms arms) {
    this.arms = arms;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled. (this requires tuning, this numebr represents the position, not speed)
  @Override
  public void execute() {
    arms.setPosition(Constants.manndiblePosition.conePos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.grab(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(arms.getPos() > Constants.manndiblePosition.conePos+2000){
    //   System.out.println("COMMANDS IS DONE");
    //   return true;
    // } else {
    //   System.out.println("COMMANDS IS NOT DONE");
    //   return false;
    // }
      if(timer.get() > 0.25){
        return true;
      } else{
        return false;
      }
  }
}
