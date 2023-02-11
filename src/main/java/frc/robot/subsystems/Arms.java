// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Arms extends SubsystemBase {
  
  
  final WPI_TalonFX armsMotor = new WPI_TalonFX(4);
  /** Creates a new ExampleSubsystem. */
  public Arms() {
    armsMotor.configFactoryDefault();
    resetEncoders();
  }

  public void resetEncoders(){
    armsMotor.setSelectedSensorPosition(0.0);
  }

  public void grab(double speed){
    armsMotor.set(speed);
  }



  @Override
  public void periodic() {
     
    
  }

  public double getPos(){
    return armsMotor.getSelectedSensorPosition();
  }

  public boolean withinRetractorDegreeRange(double degree){
    return (degree >= Constants.Intake.kRetractorMinDegree && degree <= Constants.Intake.kRetractorMaxDegree);
  }

  public void setRetractorDegree(double degrees){
    double currentDegree = getPos();
    if(withinRetractorDegreeRange(currentDegree)){
      armsMotor.set(ControlMode.Position, degrees * Constants.Intake.kRetractorOutputDegreesToInputTicks);

    }
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
