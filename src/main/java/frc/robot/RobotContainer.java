// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stuypulse.stuylib.input.gamepads.AutoGamepad;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmsOpen;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.dopeSlopeControl;
import frc.robot.constants.Ports;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DopeSlope;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DrivetrainDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final AutoGamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  
  private final Drivetrain drivetrain = new Drivetrain();

  private final DrivetrainDrive DrivetrainDrive = new DrivetrainDrive(drivetrain, driver);

  private final Arms m_arms = new Arms();

  private final Intake m_intake = new Intake();

  private final ArmsOpen m_autoCommand = new ArmsOpen(m_arms);

  private final IntakePull m_autocommand = new IntakePull(m_intake);

  private final DopeSlope dopeslope = new DopeSlope();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }
  private void configureDefaultCommands() {
    dopeslope.setDefaultCommand(new dopeSlopeControl(dopeslope, driver));
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.getLeftButton().whileTrue(new ArmsOpen(m_arms));
    driver.getTopButton().whileTrue(new IntakePull(m_intake)); 
    driver.getBottomButton().whileTrue(new IntakePush(m_intake));
  }
  
//https://docs.google.com/drawings/d/1e4qhpc7L0whN3PPnOP-MKl21IsEmKWJkSbZKH311heM/edit?usp=sharing
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
