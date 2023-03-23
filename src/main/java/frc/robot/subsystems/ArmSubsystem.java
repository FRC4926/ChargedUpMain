// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.GalacPIDController;

public class ArmSubsystem extends SubsystemBase {

  public CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN_IDs.shoulderID, MotorType.kBrushless);
  public CANSparkMax forearmMotor = new CANSparkMax(Constants.CAN_IDs.forearmID, MotorType.kBrushless);
  public CANSparkMax wristMotor = new CANSparkMax(Constants.CAN_IDs.wristID, MotorType.kBrushless);
  public CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN_IDs.intakeID, MotorType.kBrushless);

  double forearmGearRatio = 0.008;
  double shoulderGearRatio = 60;
  double wristGearRatio = 0.0052;

  public double forearmState = 0;
  public double shoulderState = 0;
  public double wristState = 0;

  public GalacPIDController pidControllerShoulder = new GalacPIDController(0.017, 0, 0, 0.01, () -> getDegreesShoulder(),
      0,
      0);
  public GalacPIDController pidControllerForearm = new GalacPIDController(0.02, 0.00, 0, 0.01, () -> getDegreesForearm(),
      0, 0);
  public GalacPIDController pidControllerWrist = new GalacPIDController(0.005, 0, 0, 0.007, () -> getDegreesWrist(),
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

    if(forearmState == Constants.ArmSetpoints.resetForearm && wristState == Constants.ArmSetpoints.resetWrist){
      pidControllerForearm.innerController.setP(0.003);
      pidControllerWrist.innerController.setP(0.007);
    }
    else if(forearmState != Constants.ArmSetpoints.resetForearm && wristState != Constants.ArmSetpoints.resetWrist){
      pidControllerForearm.innerController.setP(0.012);
      pidControllerWrist.innerController.setP(0.008);
    }
    // if(Math.abs(RobotContainer.operator.getRawAxis(0)) < 0.3 && Math.abs(RobotContainer.operator.getRawAxis(1)) < 0.3 &&
    // Math.abs(RobotContainer.operator.getRawAxis(2)) < 0.3){
      forearmMotor.set(pidControllerForearm.getEffort());
      shoulderMotor.set(pidControllerShoulder.getEffort());
      wristMotor.set(pidControllerWrist.getEffort());
    // }
  }


  public void changePID(double p){
    pidControllerForearm.innerController.setP(p);

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
    return shoulderMotor.getEncoder().getPosition();
  }

  public double getDegreesWrist(){
    return (wristMotor.getEncoder().getPosition() * 360 * wristGearRatio);
  }

  public void resetSetpoints(){
    shoulderState = 0;
    forearmState = 0;
    wristState = 0;
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
