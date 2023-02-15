// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN_IDs.shoulderID, MotorType.kBrushless);
  CANSparkMax elbowMotor = new CANSparkMax(Constants.CAN_IDs.elbowID, MotorType.kBrushless);


  /** Creates a new ArmSubsytem. */
  public ArmSubsystem() {
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setIdleMode(IdleMode.kBrake);
  }

  
  public void moveShoulder(double shoulderSpeed){
    shoulderMotor.set(shoulderSpeed);
  }

  public void moveElbow(double forearmSpeed){
    elbowMotor.set(forearmSpeed);
  }

  //stops the arm at a set position 
  public void stopAtPosition(int position){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
