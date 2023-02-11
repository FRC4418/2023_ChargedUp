// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DopeSlope extends SubsystemBase {
  /** Creates a new DopeSlope. */
  final WPI_TalonFX leftMotor = new WPI_TalonFX(02);
  final WPI_TalonFX rightMotor = new WPI_TalonFX(31);
  final AnalogPotentiometer potentiometerThing = new AnalogPotentiometer(1,100,0);

  public DopeSlope() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.setInverted(true);
  }

  public void move(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getLength());
  }

  public double getLength(){
    return potentiometerThing.get();
  }

  public double getEncoderLength(){
    return leftMotor.getSelectedSensorPosition();
  }

  public void resetEncoders(){
    leftMotor.setSelectedSensorPosition(0d);
  }
}
