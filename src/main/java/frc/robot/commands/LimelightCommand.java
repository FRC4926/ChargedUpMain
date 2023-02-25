// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class LimelightCommand extends CommandBase {
  public double XAngle;
  double angleSetpoint = -23;
  double kP = 0.007;
  double turningValue;
  double strafingValue;
  double minEffort = 0.05;
  private double error;
  /** Creates a new LimlightCommand. */
  public LimelightCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = 3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {





  
    double tx = Subsystems.limelightSubsystem.getTX();
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;
    strafingValue = (-tx * kP);
    SmartDashboard.putNumber("limelight tx", tx);
    // SmartDashboard/.putBoolean("button pressed", RobotContainer.operator2.getRawButtonReleased(1));

    if(RobotContainer.operator.getRightBumper()){ 
        if(tx > 1){
          Subsystems.driveSubsystem.drive(0, 0.2, (kP * tx) - minEffort, false);
        }
        else if(tx < 1){
          Subsystems.driveSubsystem.drive(0, -0.2, (kP * tx) + minEffort, false);
        }
    }
  }
}

