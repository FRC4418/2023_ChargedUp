// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Arms extends SubsystemBase {
  
  final WPI_TalonFX armsMotor = new WPI_TalonFX(1);
  /** Creates a new ExampleSubsystem. */
  public Arms() {
    armsMotor.configFactoryDefault();
  }

  public void grab(Number position){
    armsMotor.set(ControlMode.Position , (double)position);
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
