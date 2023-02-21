// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  public WPI_TalonFX rightElevator = new WPI_TalonFX(31);
  public WPI_TalonFX leftElevator = new WPI_TalonFX(34);
  MotorControllerGroup elevatorMotors = new MotorControllerGroup(rightElevator, leftElevator);

  public Arm() {
    //config
    rightElevator.configFactoryDefault();
    leftElevator.configFactoryDefault();

    //inversion
    rightElevator.setInverted(true);
    leftElevator.setInverted(false);

    //brakes
    rightElevator.setNeutralMode(NeutralMode.Brake);
    leftElevator.setNeutralMode(NeutralMode.Brake);
  }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      rightElevator.setSelectedSensorPosition(0);
      leftElevator.setSelectedSensorPosition(0);
    }

    public void setPID(double p, double i, double k){
      
    }

    private double nativeUnitsToDistanceMeters(double sensorCoubts){
      double motorRotations = (double)sensorCoubts / DriveConstants.kEncoderCPR;
      double wheelRoations = motorRotations / 6;
      double positionMeters = wheelRoations * (2 * Math.PI * Units.inchesToMeters(2));
      return positionMeters;
    }
  
  
    public double getAverageEncoderDistance() {
      return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }
  
    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoderDistance() {
      return -nativeUnitsToDistanceMeters(leftElevator.getSelectedSensorPosition());
    }
  
    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoderDistance() {
      return nativeUnitsToDistanceMeters(rightElevator.getSelectedSensorPosition());
    }

    public void set(){
      leftElevator.set(1);
      rightElevator.set(1);
    }

    public void inverseSet(){
      leftElevator.set(-0.15);
      rightElevator.set(-0.15);
    }
    public void stop(){
      leftElevator.set(0);
      rightElevator.set(0);
    }
    public void witchCraft(){
      leftElevator.set(ControlMode.Position, 22000);
      rightElevator.set(ControlMode.Position, 22000);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
