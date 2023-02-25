// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GalacPIDController2;

public class ArmSubsystem extends SubsystemBase {



  public  CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN_IDs.shoulderID, MotorType.kBrushless);
  public CANSparkMax forearmMotor = new CANSparkMax(Constants.CAN_IDs.elbowID, MotorType.kBrushless);
  public CANSparkMax gripMotor = new CANSparkMax(Constants.CAN_IDs.wristID, MotorType.kBrushless);
  public CANSparkMax padMotor = new CANSparkMax(Constants.CAN_IDs.gripID, MotorType.kBrushless);

  private double highConeAngleShoulder = 127;
  public double highConeAngleForearm = 64;
  private double lowConeAngleShoulder = 85;
  private double lowConeAngleForearm = 127;
  private double highCubeAngleShoulder = 113;
  private double highCubeAngleForearm = 195;
  private double lowCubeAngleShoulder = 81;
  private double lowCubeAngleForearm = 158;
  private double groundAngleShoulder = 80;
  private double groundAngleForearm = -45;
  private double collectAngleShoulder = 125;
  private double collectAngleForearm = -180;
  private double substationShoulderAngle =909;
  private double substationForearmAngle = 909;
  private double moveBackShoulderAngle = 102;
  // Check wirst range compatibility
  private double highConeAnglePad = -32.5;
  private double lowConeAnglePad = -97;
  private double highCubeAnglePad = 45;
  private double lowCubeAnglePad = 173;
  private double groundAnglePad = 0;
  private double collectAnglePad = 0;
  private double cubeGripAngle = 0;
  private double gripReleaseAngle = 90;
  private double coneGripAngle = 0;
  private double substationPadAngle = 909;
  private double collectAngleShoulderCone = 125;
  private double collectAngleForearmCone = -180;
  private double collectAngleShoulderCube = 125;
  private double collectAngleForearmCube = -180;


  double p = 0.005;
  double i = 0.00;
  double d = 0.00;

  double pShoulder = 0.025;
  double iShoulder = 0.000;
  double dShoulder = 0;

  double pReset = 0.0005;
  double iReset = 0;
  double dReset = 0;

  double minEffort = 0.1;
  double forearmGearRatio = 0.008; // 1/125
  double shoulderGearRatio = 60;
  double PadGearRatio = 1;
  double gripConversionFactor = 1;

  // creates lambdas for all 4 arm motors (allows them to update continuously)
  Supplier<Double> getShoulderAngle = () -> getDegreesShoulder();
  public Supplier<Double> getForearmAngle = () -> getDegreesForearm();
  Supplier<Double> getPadAngle = () -> getDegreesPad();
  Supplier<Double> getGripAngle = () -> getDegreesGrip();

  // creates PID controllers for all 4 arm motors (default set point is
  // highConeAngle)
  public GalacPIDController2 pidControllerShoulder = new GalacPIDController2(pShoulder, iShoulder, dShoulder, minEffort, () -> getDegreesShoulder(),
      0,
      0.1);
  public GalacPIDController2 pidControllerForearm = new GalacPIDController2(p, i, d, minEffort, () -> getDegreesForearm(),
      0, 0.1);
  public GalacPIDController2 pidControllerPad = new GalacPIDController2(p, i, d, minEffort, getPadAngle, highConeAnglePad,
      1);
  public GalacPIDController2 pidControllerGrip = new GalacPIDController2(p, i, d, minEffort, getGripAngle, 0, 1);

  /** Creates a new ArmSubsytem. */
  public ArmSubsystem() {
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    forearmMotor.setIdleMode(IdleMode.kBrake);
    padMotor.setIdleMode(IdleMode.kBrake);
    gripMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setSmartCurrentLimit(60);
    forearmMotor.setSmartCurrentLimit(60);
    gripMotor.setSmartCurrentLimit(60);
    padMotor.setSmartCurrentLimit(60);
  }

  // sets each arm motor to a given effort
  public void moveShoulder(double shoulderSpeed) {
    shoulderMotor.set(shoulderSpeed);
  }

  public void moveForearm(double forearmSpeed) {
    forearmMotor.set(forearmSpeed);
  }

  public void movePad(double PadSpeed) {
    padMotor.set(PadSpeed);
  }

  public void moveGrip(double gripSpeed) {
    gripMotor.set(gripSpeed);
  }

  public double getShoulderOutputVoltage(){
    return shoulderMotor.getBusVoltage();
  }

  public double getForearmOutputVoltage(){
    return forearmMotor.getBusVoltage();
  }

  // gets the angle of each arm motor in degrees (need to factor in gear ratio),
  // grip uses position, need conversion factor
  public double getDegreesForearm() {
    return (-forearmMotor.getEncoder().getPosition() * 360 * forearmGearRatio) - 90 ;
  }

  public double getDegreesShoulder() {
    return -shoulderMotor.getEncoder().getPosition()+90;
  }

  public double getDegreesPad() {
    return padMotor.getEncoder().getPosition() * 360;
  }

  public double getDegreesGrip() {
    return gripMotor.getEncoder().getPosition() * 360+90;
  }

  // changes set points based off target and object
  // sets the proper effort for each motor using PID
  public void moveToUpperCone() {
    pidControllerForearm.innerController.setP(p);
    pidControllerForearm.innerController.setI(i);
    pidControllerForearm.innerController.setD(d);
    pidControllerShoulder.innerController.setP(pShoulder);
    pidControllerShoulder.innerController.setI(iShoulder);
    pidControllerShoulder.innerController.setD(dShoulder);
    pidControllerForearm.innerController.setTolerance(0.1);
    pidControllerForearm.setMinEffort(minEffort);

  
  }

  public void moveToLowerCone() {
    pidControllerForearm.innerController.setP(p);
    pidControllerForearm.innerController.setI(i);
    pidControllerForearm.innerController.setD(d);
    pidControllerShoulder.innerController.setP(pShoulder);
    pidControllerShoulder.innerController.setI(iShoulder);
    pidControllerShoulder.innerController.setD(dShoulder);
    pidControllerForearm.innerController.setTolerance(0.1);
    pidControllerForearm.setMinEffort(minEffort);

    if (pidControllerShoulder.getSetpoint() != lowConeAngleShoulder) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(lowConeAngleForearm);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(lowConeAngleShoulder);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

    pidControllerPad.setSetpoint(lowConeAnglePad);
    // padMotor.set(pidControllerPad.getEffort());
    
  }


  public void moveToUpperBox() {
    pidControllerForearm.innerController.setP(p);
    pidControllerForearm.innerController.setI(i);
    pidControllerForearm.innerController.setD(d);
    pidControllerShoulder.innerController.setP(pShoulder);
    pidControllerShoulder.innerController.setI(iShoulder);
    pidControllerShoulder.innerController.setD(dShoulder);
    pidControllerForearm.innerController.setTolerance(0.1);
    pidControllerForearm.setMinEffort(minEffort);


    
     if (pidControllerShoulder.getSetpoint() != highCubeAngleShoulder) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(highCubeAngleForearm);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(highCubeAngleShoulder);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

    pidControllerPad.setSetpoint(highCubeAnglePad);

    // padMotor.set(pidControllerPad.getEffort());
  }

  public void moveToLowerBox() {
    pidControllerForearm.innerController.setP(p);
    pidControllerForearm.innerController.setI(i);
    pidControllerForearm.innerController.setD(d);
    pidControllerShoulder.innerController.setP(pShoulder);
    pidControllerShoulder.innerController.setI(iShoulder);
    pidControllerShoulder.innerController.setD(dShoulder);
    pidControllerForearm.innerController.setTolerance(0.1);
    pidControllerForearm.setMinEffort(minEffort);

    if (pidControllerShoulder.getSetpoint() != lowCubeAngleShoulder) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(lowCubeAngleForearm);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(lowCubeAngleShoulder);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

    pidControllerPad.setSetpoint(lowCubeAnglePad);
    // padMotor.set(pidControllerPad.getEffort());
    
  }

  public void moveToGround() {
    // pidControllerForearm.innerController.setP(p);
    // pidControllerForearm.innerController.setI(i);
    // pidControllerForearm.innerController.setD(d);
    pidControllerShoulder.innerController.setP(pShoulder);
    pidControllerShoulder.innerController.setI(iShoulder);
    pidControllerShoulder.innerController.setD(dShoulder);
    pidControllerForearm.innerController.setTolerance(0.1);
    pidControllerForearm.setMinEffort(minEffort);

    if (pidControllerShoulder.getSetpoint() != groundAngleShoulder) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(groundAngleForearm);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(groundAngleShoulder);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

    pidControllerPad.setSetpoint(groundAnglePad);
    // true = cone, false = cube
   
    // padMotor.set(pidControllerPad.getEffort());
  }

  public void moveToReset(){
    pidControllerForearm.innerController.setP(0.004);
    pidControllerForearm.innerController.setI(0);
    pidControllerForearm.innerController.setD(0);
    pidControllerForearm.innerController.setTolerance(1);

    pidControllerShoulder.innerController.setP(0.01);
    pidControllerShoulder.innerController.setI(0);
    pidControllerShoulder.innerController.setD(0);
    if (pidControllerShoulder.getSetpoint() != collectAngleShoulderCone) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(collectAngleForearmCone);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(collectAngleShoulderCone);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

  }


  public void moveToSubstation(){
    if (pidControllerShoulder.getSetpoint() != substationShoulderAngle) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(substationForearmAngle);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(substationShoulderAngle);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }

    pidControllerPad.setSetpoint(substationPadAngle);

    // padMotor.set(pidControllerPad.getEffort());

  }
  

  public void moveToCollectCone() {
    if (pidControllerShoulder.getSetpoint() != collectAngleShoulderCone) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(collectAngleForearmCone);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(collectAngleShoulderCone);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }
 
  }
  public void moveToCollectCube(){
    if (pidControllerShoulder.getSetpoint() != collectAngleShoulderCube) {
      pidControllerShoulder.setSetpoint(moveBackShoulderAngle);
    }

    if (pidControllerShoulder.isFinished()) {
      pidControllerForearm.setSetpoint(collectAngleForearmCube);
      forearmMotor.set(pidControllerForearm.getEffort() * -1);
      if (pidControllerForearm.isFinished()) {
        pidControllerShoulder.setSetpoint(collectAngleShoulderCube);
      }
    } else {
      shoulderMotor.set(pidControllerShoulder.getEffort() * -1);
    }
  }

  public void grabCone() {
    pidControllerGrip.setSetpoint(coneGripAngle);
    gripMotor.set(pidControllerGrip.getEffort()*-1);
  }

  public void grabCube() {
    pidControllerGrip.setSetpoint(cubeGripAngle);
    gripMotor.set(pidControllerGrip.getEffort()*-1);
  }

  public void release() {
    pidControllerGrip.setSetpoint(gripReleaseAngle);
    gripMotor.set(pidControllerGrip.getEffort()*-1);
  }

  public boolean hasReachedTarget() {
    return (pidControllerShoulder.isFinished() && pidControllerForearm.isFinished() && pidControllerPad.isFinished());
  }
  
  // stops the arm at a set position
  public void stopAtPosition(int position) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    shoulderMotor.getEncoder().setPosition(0);
    forearmMotor.getEncoder().setPosition(0);
    gripMotor.getEncoder().setPosition(0);
    padMotor.getEncoder().setPosition(0);
  }
}
