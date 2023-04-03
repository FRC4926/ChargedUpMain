// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.GalacPIDController;

public class ArmSubsystem extends SubsystemBase {

  public CANSparkMax shoulderMotor = new CANSparkMax(Constants.CanIDs.shoulderID, MotorType.kBrushless);
  public CANSparkMax forearmMotor = new CANSparkMax(Constants.CanIDs.forearmID, MotorType.kBrushless);
  public CANSparkMax wristMotor = new CANSparkMax(Constants.CanIDs.wristID, MotorType.kBrushless);
  public CANSparkMax intakeMotor = new CANSparkMax(Constants.CanIDs.intakeID, MotorType.kBrushless);

  double forearmGearRatio = 0.005208;
  double shoulderGearRatio = 0.008;
  double wristGearRatio = 0.0052;

  public double forearmState = 0;
  public double shoulderState = 0;
  public double wristState = 0;

  public double ffVelocity = 2.5;

  public ArmFeedforward ff = new ArmFeedforward(0, 1.2, 0.2);


  public GalacPIDController pidControllerShoulder = new GalacPIDController(0.03, 0, 0, 0.01, () -> getDegreesShoulder(),
      0,
      0);
  public GalacPIDController pidControllerForearm = new GalacPIDController(0.2, 0.00, 0, 0.01, () -> getDegreesForearm(),
      0, 0);
  public GalacPIDController pidControllerWrist = new GalacPIDController(0.014, 0, 0, 0.007, () -> getDegreesWrist(),
      0, 0);


  /** Creates a new ArmSubsytem. */
  public ArmSubsystem() {
    setBrakeArm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidControllerForearm.setSetpoint(forearmState);
    pidControllerShoulder.setSetpoint(shoulderState);
    pidControllerWrist.setSetpoint(wristState);
   
    if(RobotContainer.isAutomated){
      pidControllerForearm.innerController.setP(0.2);
      if(forearmState == Constants.ArmSetpoints.highForearm || forearmState == Constants.ArmSetpoints.singleSubForearm){
        if(!forearmComplete()){
          forearmMotor.setVoltage(pidControllerForearm.innerController.calculate(getDegreesForearm(), forearmState) + ff.calculate(Math.toRadians(forearmState)+4.71, ffVelocity));
        }
        else{
          forearmMotor.setVoltage(pidControllerForearm.innerController.calculate(getDegreesForearm(), forearmState) + ff.calculate(Math.toRadians(forearmState)+4.71, ffVelocity));
          shoulderMotor.set(pidControllerShoulder.getEffort());
          wristMotor.set(pidControllerWrist.getEffort());
        }
      }
      else if(forearmState == 0){
        pidControllerForearm.innerController.setP(0.1);
        if(!wristComplete()){
          
          wristMotor.set(pidControllerWrist.getEffort());
        }
        else{
          forearmMotor.setVoltage(pidControllerForearm.innerController.calculate(getDegreesForearm(), forearmState) + ff.calculate(Math.toRadians(forearmState)+4.71, 1));
          shoulderMotor.set(pidControllerShoulder.getEffort());
          wristMotor.set(pidControllerWrist.getEffort());
        }
      }
      else{
        forearmMotor.setVoltage(pidControllerForearm.innerController.calculate(getDegreesForearm(), forearmState) + ff.calculate(Math.toRadians(forearmState)+4.71, ffVelocity));
        shoulderMotor.set(pidControllerShoulder.getEffort());
        wristMotor.set(pidControllerWrist.getEffort());
      }
    }
  }


  public void moveShoulder(double shoulderSpeed) {
    shoulderMotor.set(shoulderSpeed);
  }

  public void moveForearm(double forearmSpeed) {
    forearmMotor.set(forearmSpeed);
  }

  public void moveWrist(double wristSpeed) {
    wristMotor.set(wristSpeed);
  }

  public void moveIntake(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public double getDegreesForearm() {
    return (forearmMotor.getEncoder().getPosition() * 360 * forearmGearRatio);
  }

  public double getDegreesShoulder() {
    return shoulderMotor.getEncoder().getPosition() * 360 * shoulderGearRatio;
  }

  public double getDegreesWrist(){
    return (wristMotor.getEncoder().getPosition() * 360 * wristGearRatio);
  }

  public void resetSetpoints(){
    shoulderState = Constants.ArmSetpoints.resetShoulder;
    forearmState = Constants.ArmSetpoints.resetForearm;
    wristState = Constants.ArmSetpoints.resetWrist;
  }

  public boolean hasReachedTarget() {
    return 
      (Math.abs(getDegreesForearm() - pidControllerForearm.getSetpoint()) < 12 
    && Math.abs(getDegreesShoulder() - pidControllerShoulder.getSetpoint()) < 5 
    && Math.abs(getDegreesWrist() - pidControllerWrist.getSetpoint()) < 5);
  }

  public void resetEncoders() {
    shoulderMotor.getEncoder().setPosition(0);
    forearmMotor.getEncoder().setPosition(0);
    wristMotor.getEncoder().setPosition(0);
  }

  public boolean forearmComplete(){
    return Math.abs(getDegreesForearm()) > 70;
  }

  public boolean wristComplete(){
    return getDegreesWrist() < 70;
  }

  public void setBrakeArm(){
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    forearmMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setIdleMode(IdleMode.kBrake);
  }
  
  public void setCoastArm(){
    shoulderMotor.setIdleMode(IdleMode.kCoast);
    forearmMotor.setIdleMode(IdleMode.kCoast);
    wristMotor.setIdleMode(IdleMode.kCoast);
  }

}
