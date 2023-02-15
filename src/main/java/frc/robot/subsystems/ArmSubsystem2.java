// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GalacPIDController2;

public class ArmSubsystem2 extends SubsystemBase {

  public CANSparkMax shoulderMotor = new CANSparkMax(15, MotorType.kBrushless);
  public CANSparkMax elbowMotor = new CANSparkMax(16, MotorType.kBrushless);
  public CANSparkMax wristMotor = new CANSparkMax(13, MotorType.kBrushless);
  public CANSparkMax gripMotor = new CANSparkMax(14, MotorType.kBrushless);
  private double offsetShoulder = 0;
  private double offsetElbow = 0;
  private double offsetWrist = 0;
  private double offsetGrip = 0;
  private double highConeAngleShoulder = 53;
  private double highConeAngleElbow = 137;
  private double lowConeAngleShoulder = 92;
  private double lowConeAngleElbow = 90;
  private double highCubeAngleShoulder = 909;
  private double highCubeAngleElbow = 909;
  private double lowCubeAngleShoulder = 909;
  private double lowCubeAngleElbow = 909;
  private double groundAngleShoulder = 116;
  private double groundAngleElbow = 75;

  private double highConeAngleWrist = 135;
  private double lowConeAngleWrist = 135;
  private double highCubeAngleWrist = 909;
  private double lowCubeAngleWrist = 909;
  private double groundAngleWrist = 45;
  private double cubeGripAngle = 909;
  private double coneGripAngle = 909;

  public RelativeEncoder shoulderEncoder;
   public RelativeEncoder elbowEncoder;
   public RelativeEncoder wristEncoder;
   public RelativeEncoder gripEncoder;
  
  double p = 0.001;
  double i = 0.001;
  double d = 0.001;
  double minEffort = 0.05;
  double elbowGearRatio = 1;
  double shoulderGearRatio = 1;
  double wristGearRatio = 1;
  double gripGearRatio = 1;

  // creates lambdas for all 4 arm motors (allows them to update continuously)
   Supplier<Double> getShoulderAngle = () -> getDegreesShoulder();
   Supplier<Double> getElbowAngle = () -> getDegreesElbow();
   Supplier<Double> getWristAngle = () -> getDegreesWrist();
   Supplier<Double> getGripAngle = () -> getDegreesGrip();
  
   //creates PID controllers for all 4 arm motors (default set point is highConeAngle)
   GalacPIDController2 pidControllerShoulder = new GalacPIDController2(p, i, d, minEffort , getShoulderAngle, highConeAngleShoulder,
   1);
   GalacPIDController2 pidControllerElbow = new GalacPIDController2(p, i, d, minEffort,  getElbowAngle, highConeAngleElbow, 1);
   GalacPIDController2 pidControllerWrist = new GalacPIDController2(p, i, d, minEffort , getWristAngle, highConeAngleWrist,
   1);
   GalacPIDController2 pidControllerGrip = new GalacPIDController2(p, i, d, minEffort,  getGripAngle, coneGripAngle, 1);

  /** Creates a new ArmSubsytem. */
  public ArmSubsystem2() {
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setIdleMode(IdleMode.kBrake);
    gripMotor.setIdleMode(IdleMode.kBrake);

    shoulderEncoder = shoulderMotor.getEncoder();
    elbowEncoder = elbowMotor.getEncoder();
    wristEncoder = wristMotor.getEncoder();
    gripEncoder = gripMotor.getEncoder();

    resetEncoders();
  }


  // sets each arm motor to a given effort
  public void moveShoulder(double shoulderSpeed){
    shoulderMotor.set(shoulderSpeed);
  }
  public void moveElbow(double elbowSpeed){
    elbowMotor.set(elbowSpeed);
  }
  public void moveWrist(double wristSpeed){
    wristMotor.set(wristSpeed);
  }
  public void moveGrip(double gripSpeed){
    gripMotor.set(gripSpeed);
  }

  // gets the angle of each arm motor in degrees (need to factor in gear ratio), grip uses position, need conversion factor
  public double getDegreesElbow(){
    return elbowMotor.getEncoder().getPosition()/360 + offsetElbow;
  }
  public double getDegreesShoulder(){
    return shoulderMotor.getEncoder().getPosition()/360 + offsetShoulder;
  }
  public double getDegreesWrist(){
    return wristMotor.getEncoder().getPosition()/360 + offsetWrist;
  }
  public double getDegreesGrip(){
    return gripMotor.getEncoder().getPosition()/360 + offsetGrip;
  }

  
  // changes set points based off target and object
  // sets the proper effort for each motor using PID
  public void moveToUpperCone(){
  pidControllerShoulder.setSetpoint(highConeAngleShoulder);
  pidControllerElbow.setSetpoint(highConeAngleElbow);
  elbowMotor.set(pidControllerElbow.getEffort());
  shoulderMotor.set(pidControllerShoulder.getEffort());

  pidControllerWrist.setSetpoint(highConeAngleWrist);
  pidControllerGrip.setSetpoint(coneGripAngle);
  wristMotor.set(pidControllerWrist.getEffort());
  gripMotor.set(pidControllerGrip.getEffort());
  }
  public void moveToLowerCone(){
  pidControllerShoulder.setSetpoint(lowConeAngleShoulder);
  pidControllerElbow.setSetpoint(lowConeAngleElbow);
  elbowMotor.set(pidControllerElbow.getEffort());
  shoulderMotor.set(pidControllerShoulder.getEffort());

  pidControllerWrist.setSetpoint(lowConeAngleWrist);
  pidControllerGrip.setSetpoint(coneGripAngle);
  wristMotor.set(pidControllerWrist.getEffort());
  gripMotor.set(pidControllerGrip.getEffort());
  }
  public void moveToUpperBox(){
    pidControllerShoulder.setSetpoint(highCubeAngleShoulder);
    pidControllerElbow.setSetpoint(highCubeAngleElbow);
    elbowMotor.set(pidControllerElbow.getEffort());
    shoulderMotor.set(pidControllerShoulder.getEffort());
    
    pidControllerWrist.setSetpoint(highCubeAngleWrist);
    pidControllerGrip.setSetpoint(cubeGripAngle);
    wristMotor.set(pidControllerWrist.getEffort());
    gripMotor.set(pidControllerGrip.getEffort());
  }
  public void moveToLowerBox(){
    pidControllerShoulder.setSetpoint(lowCubeAngleShoulder);
    pidControllerElbow.setSetpoint(lowCubeAngleElbow);
    elbowMotor.set(pidControllerElbow.getEffort());
    shoulderMotor.set(pidControllerShoulder.getEffort());
    
    pidControllerWrist.setSetpoint(lowCubeAngleWrist);
    pidControllerGrip.setSetpoint(cubeGripAngle);
    wristMotor.set(pidControllerWrist.getEffort());
    gripMotor.set(pidControllerGrip.getEffort());
  }

  public void moveToGround(boolean object){
    pidControllerShoulder.setSetpoint(groundAngleShoulder);
    pidControllerElbow.setSetpoint(groundAngleElbow);
    elbowMotor.set(pidControllerElbow.getEffort());
    shoulderMotor.set(pidControllerShoulder.getEffort());

    pidControllerWrist.setSetpoint(groundAngleWrist);
    
    // true = cone, false = cube
    if (object) {
      pidControllerGrip.setSetpoint(coneGripAngle);
    } else {
      pidControllerGrip.setSetpoint(cubeGripAngle);
    }
    wristMotor.set(pidControllerWrist.getEffort());
    gripMotor.set(pidControllerGrip.getEffort());
  }

  public boolean hasReachedTarget(boolean object, int level)
  {
    double shoulderTarget = 0;
    double elbowTarget = 0;
    double wristTarget = 0;
    double gripTarget = 0;
    switch (level) {
      case 0:
        shoulderTarget = groundAngleShoulder;
        elbowTarget = groundAngleElbow;
        wristTarget = groundAngleWrist;
        if(object)
          gripTarget = coneGripAngle;
        else
          gripTarget = cubeGripAngle;
        break;
      case 1:
        if(object)
        {
          shoulderTarget = lowConeAngleShoulder;
          elbowTarget = lowConeAngleElbow;
          wristTarget = lowConeAngleWrist;
          gripTarget = coneGripAngle;
        }
        else
        {
          shoulderTarget = lowCubeAngleShoulder;
          elbowTarget = lowCubeAngleElbow;
          wristTarget = lowCubeAngleWrist;
          gripTarget = cubeGripAngle;
        }
        break;
      case 2:
      if(object)
      {
        shoulderTarget = highConeAngleShoulder;
        elbowTarget = highConeAngleElbow;
        wristTarget = highConeAngleWrist;
        gripTarget = coneGripAngle;
      }
      else
      {
        shoulderTarget = highCubeAngleShoulder;
        elbowTarget = highCubeAngleElbow;
        wristTarget = highCubeAngleWrist;
        gripTarget = cubeGripAngle;
      }
        break;
      default:
        break;
    }
    //tolerances not tested
    return (Math.abs(getDegreesShoulder()-shoulderTarget)<1) && (Math.abs(getDegreesElbow()-elbowTarget)<1) && (Math.abs(getDegreesWrist()-wristTarget)<1 && (Math.abs(getDegreesGrip()-gripTarget)<.2));
  }

  public void resetEncoders() {
    
    shoulderEncoder.setPosition(0);
    elbowEncoder.setPosition(0);
    wristEncoder.setPosition(0);
    gripEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}