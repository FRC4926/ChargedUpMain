// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GalacPIDController;

public class ArmSubsystem extends SubsystemBase {

  // public CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN_IDs.shoulderID, MotorType.kBrushless);
  // public CANSparkMax forearmMotor = new CANSparkMax(Constants.CAN_IDs.elbowID, MotorType.kBrushless);
  public WPI_TalonSRX wristMotor = new WPI_TalonSRX(13);
  public WPI_TalonSRX gripMotor = new WPI_TalonSRX(14);


  double pForearm = 0.01;
  double iForearm = 0.00;
  double dForearm = 0.00;

  double pShoulder = 0.0055;
  double iShoulder = 0.000;
  double dShoulder = 0;

  double minEffort = 0.005;
  double forearmGearRatio = 0.008; // 1/125
  double shoulderGearRatio = 60;

  public double forearmState = -90;
  public double shoulderState = 90;


  // creates lambdas for all 4 arm motors (allows them to update continuously)
  public Supplier<Double> getShoulderAngle = () -> getDegreesShoulder();
  public Supplier<Double> getForearmAngle = () -> getDegreesForearm();

  // creates PID controllers for both arm motors (default set point is
  // highConeAngle)
  public GalacPIDController pidControllerShoulder = new GalacPIDController(pShoulder, iShoulder, dShoulder, minEffort, () -> getDegreesShoulder(),
      0,
      0);
  public GalacPIDController pidControllerForearm = new GalacPIDController(pForearm, iForearm, dForearm, minEffort, () -> getDegreesForearm(),
      0, 0);


  /** Creates a new ArmSubsytem. */
  public ArmSubsystem() {
    // shoulderMotor.setIdleMode(IdleMode.kBrake);
    // forearmMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    gripMotor.setNeutralMode(NeutralMode.Brake);

    // shoulderMotor.setSmartCurrentLimit(60);
    // forearmMotor.setSmartCurrentLimit(60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidControllerForearm.setSetpoint(forearmState);
    pidControllerShoulder.setSetpoint(shoulderState);

    moveForearm(pidControllerForearm.getEffort());
    moveShoulder(pidControllerShoulder.getEffort());
  }

  // sets each arm motor to a given effort
  public void moveShoulder(double shoulderSpeed) {
    // shoulderMotor.set(shoulderSpeed);
  }

  public void moveForearm(double forearmSpeed) {
    // forearmMotor.set(forearmSpeed);
  }

  public void moveWrist(double wristSpeed) {
    wristMotor.set(wristSpeed);
  }

  public void moveGrip(double gripSpeed) {
    gripMotor.set(gripSpeed);
  }

  public double getDegreesForearm() {
    // return (-forearmMotor.getEncoder().getPosition() * 360 * forearmGearRatio) - 90 ;
    return 0;
  }

  public double getDegreesShoulder() {
    // return -shoulderMotor.getEncoder().getPosition()+90;
    return 0;
  }

  public boolean hasReachedTarget() {
    return (Math.abs(getDegreesForearm() - pidControllerForearm.getSetpoint()) < 9.5 && Math.abs(getDegreesShoulder() - pidControllerShoulder.getSetpoint()) < 5);
  }

  public void resetEncoders() {
    // shoulderMotor.getEncoder().setPosition(0);
    // forearmMotor.getEncoder().setPosition(0);
  }
}
