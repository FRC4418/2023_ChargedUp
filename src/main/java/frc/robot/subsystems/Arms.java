// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Arms extends SubsystemBase {
  
  
  final WPI_TalonFX armsMotor = new WPI_TalonFX(21);
  /** Creates a new ExampleSubsystem. */
  public Arms() {
    armsMotor.configFactoryDefault();
    resetEncoders();
    armsMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stop(){
    armsMotor.set(0);
  }

  public void resetEncoders(){
    armsMotor.setSelectedSensorPosition(0.0);
  }

  public void grab(double speed){
    armsMotor.set(speed);
  }



  @Override
  public void periodic() {}

  public void resetEncoder(){
    armsMotor.setSelectedSensorPosition(0);
  }

  public double getPos(){
    return armsMotor.getSelectedSensorPosition();
  }


  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
