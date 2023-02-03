// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake extends SubsystemBase {
  final WPI_TalonFX wheel1Motor = new WPI_TalonFX(2);
  final WPI_TalonFX wheel2Motor = new WPI_TalonFX(3);
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    wheel1Motor.configFactoryDefault();
    wheel2Motor.configFactoryDefault();
  }

  public void intake(Number speed){
    wheel1Motor.set((double)speed);
    wheel2Motor.set(-(double)speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
