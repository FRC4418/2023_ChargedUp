// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.nimbus.State;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private PIDController pidController = new PIDController(0.2,0.1,0.1);
  
  public WPI_TalonFX rightElevator = new WPI_TalonFX(21);
  public WPI_TalonFX leftElevator = new WPI_TalonFX(20);
  
  public final SimpleMotorFeedforward armFeedForward = new SimpleMotorFeedforward(1.0,1.0);
  private TrapezoidProfile.Constraints m_Constraints = 
    new TrapezoidProfile.Constraints(0.5,0.3);
  private TrapezoidProfile profile;
  private static final double maxVel = 10;
  private static final double maxAccel = 10;
  private static final int kTimeOut = 30;
  
  


  public Arm() {
    rightElevator.setInverted(true);

    leftElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , kTimeOut);
    rightElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , kTimeOut);
    
    leftElevator.configMotionCruiseVelocity(maxVel,kTimeOut);
    rightElevator.configMotionCruiseVelocity(maxVel, kTimeOut);

    leftElevator.configMotionAcceleration(maxAccel,kTimeOut);
    rightElevator.configMotionAcceleration(maxAccel, kTimeOut);
    //config
    rightElevator.configFactoryDefault();
    leftElevator.configFactoryDefault();

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
      leftElevator.set(0.5);
      rightElevator.set(0.5);
    }

    public void inverseSet(){
      leftElevator.set(-0.15);
      rightElevator.set(-0.15);
    }
    public void stop(){
      leftElevator.set(0);
      rightElevator.set(0);
    }
    public void setArmPosition(double position){
      int positionTicks = (int) (position * 566);

      TrapezoidProfile profile = new TrapezoidProfile(
        new Constraints(maxVel,maxAccel),
        new TrapezoidProfile.State(leftElevator.getSelectedSensorPosition(), 0),
        new TrapezoidProfile.State(positionTicks, 0)
);
      leftElevator.set(ControlMode.MotionMagic, profile.calculate(leftElevator.getSelectedSensorPosition()).position);
      rightElevator.set(ControlMode.MotionMagic, profile.calculate(rightElevator.getSelectedSensorPosition()).position);
}
public void log() {
  SmartDashboard.putNumber("Arm Left Position", leftElevator.getSelectedSensorPosition());
  SmartDashboard.putNumber("Arm Right Position", rightElevator.getSelectedSensorPosition());
  SmartDashboard.putNumber("Arm Left Velocity", leftElevator.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Arm Right Velocity", rightElevator.getSelectedSensorVelocity());
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
