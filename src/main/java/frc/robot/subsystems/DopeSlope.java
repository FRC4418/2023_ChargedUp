// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DopeSlope extends SubsystemBase {
  /** Creates a new DopeSlope. */
  final WPI_TalonFX leftMotor = new WPI_TalonFX(1);
  final WPI_TalonFX rightMotor = new WPI_TalonFX(4);

  public DopeSlope() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.setInverted(true);
  }
  public void FunnyMethod(Number yval){
    leftMotor.set((double)yval/2);
    rightMotor.set((double)yval/2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
