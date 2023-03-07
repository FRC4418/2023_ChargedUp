// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class Arms extends SubsystemBase {

  private int peakVelocityUp = 273600;
  private final double percentOfPeakUp = .75;
  private final double upkF = (percentOfPeakUp * 2048) / (peakVelocityUp * percentOfPeakUp);
  private final double cruiseVelocityAccelUp = peakVelocityUp * percentOfPeakUp;

  private int peakVelocityDown = 3090;
  private final double percentOfPeakDown = .35;
  private final double downkF = (percentOfPeakDown * 2048) / (peakVelocityDown * percentOfPeakDown);
  private final double cruiseVelocityAccelDown = peakVelocityDown * percentOfPeakDown;

  
  
  final WPI_TalonFX armsMotor = new WPI_TalonFX(21);
  /** Creates a new ExampleSubsystem. */
  public Arms() {
    armsMotor.configFactoryDefault();
    armsMotor.setSelectedSensorPosition(0);
    armsMotor.setNeutralMode(NeutralMode.Brake);

    armsMotor.config_kF(0, 0.1, 0);
		armsMotor.config_kP(0, 0.06030624264, 0);
		armsMotor.config_kI(0, 0, 0);
		armsMotor.config_kD(0, 0, 0);

    armsMotor.config_kF(1, 0.1, 0);
		armsMotor.config_kP(1, 0.1265760198, 0);
		armsMotor.config_kI(1, 0, 0);
		armsMotor.config_kD(1, 0, 0);

    armsMotor.configMotionSCurveStrength(2);


  }
  public void manageMotion(double targetPosition) {
    double currentPosition = armsMotor.getSelectedSensorPosition();

    // going up
    if(currentPosition < targetPosition) {

      // set accel and velocity for going up
      armsMotor.configMotionAcceleration(cruiseVelocityAccelUp, 0);
      armsMotor.configMotionCruiseVelocity(cruiseVelocityAccelUp, 0);

      // select the up gains
      armsMotor.selectProfileSlot(0, 0);
      SmartDashboard.putBoolean("Going Up or Down", true);

    } else {
      
      // set accel and velocity for going down
      armsMotor.configMotionAcceleration(cruiseVelocityAccelDown, 0);
      armsMotor.configMotionCruiseVelocity(cruiseVelocityAccelDown, 0);

      // select the down gains
      armsMotor.selectProfileSlot(1, 0);
      SmartDashboard.putBoolean("Going Up or Down", false);

    }

  }

  public void setPosition(double position) {
    manageMotion(position);
    armsMotor.set(ControlMode.MotionMagic, position);
}


  public void goToHome() {
    armsMotor.set(TalonFXControlMode.Position, 100);
    armsMotor.set(TalonFXControlMode.Position, 100);
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
  public void periodic() {
    SmartDashboard.putNumber("Manndible Position", armsMotor.getSelectedSensorPosition());
  }

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
