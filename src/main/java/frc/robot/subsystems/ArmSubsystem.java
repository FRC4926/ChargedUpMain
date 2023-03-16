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
import frc.robot.utils.GalacPIDController;

public class ArmSubsystem extends SubsystemBase {

  public CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN_IDs.shoulderID, MotorType.kBrushless);
  public CANSparkMax forearmMotor = new CANSparkMax(Constants.CAN_IDs.forearmID, MotorType.kBrushless);
  public CANSparkMax wristMotor = new CANSparkMax(Constants.CAN_IDs.wristID, MotorType.kBrushless);
  public CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN_IDs.intakeID, MotorType.kBrushless);

  double forearmGearRatio = 0.008;
  double shoulderGearRatio = 60;
  double wristGearRatio = 192;

  public double forearmState = 0;
  public double shoulderState = 0;
  public double wristState = 0;

  public GalacPIDController pidControllerShoulder = new GalacPIDController(0, 0, 0, 0.005, () -> getDegreesShoulder(),
      0,
      0);
  public GalacPIDController pidControllerForearm = new GalacPIDController(0, 0, 0, 0.005, () -> getDegreesForearm(),
      0, 0);
  
  public GalacPIDController pidControllerWrist = new GalacPIDController(0, 0, 0, 0.005, () -> getDegreesWrist(),
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

    moveForearm(pidControllerForearm.getEffort());
    moveShoulder(pidControllerShoulder.getEffort());
    moveWrist(pidControllerWrist.getEffort());
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
    return (-forearmMotor.getEncoder().getPosition() * 360 * forearmGearRatio) - 90;
  }

  public double getDegreesShoulder() {
    return -shoulderMotor.getEncoder().getPosition()+90;
  }

  public double getDegreesWrist(){
    return (-wristMotor.getEncoder().getPosition() * 360 * wristGearRatio) - 90;
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
