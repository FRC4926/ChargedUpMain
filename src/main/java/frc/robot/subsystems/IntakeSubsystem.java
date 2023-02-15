// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax leftIntake;
  CANSparkMax rightIntake;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    leftIntake = new CANSparkMax(Constants.CAN_IDs.leftIntakeID, MotorType.kBrushless);
    rightIntake = new CANSparkMax(Constants.CAN_IDs.rightIntakeID, MotorType.kBrushless);
  }

  public void runIntake(double speed){
    leftIntake.set(-speed);
    rightIntake.set(speed);
  }

  public void stopIntake(){
    leftIntake.set(0);
    rightIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
