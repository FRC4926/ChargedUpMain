// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController2;

public class BalanceCommand extends CommandBase {
  GalacPIDController2 pidController;
  double balanceSetpoint = 0;
  double kP = 0.00665;
  double kI = 0.00001;
  double kD = 0.00001;
  double effort;

  double angleSetpoint = 0;
  double kTurn = 0.007;
  double turningValue;


  /** Creates a new AutoBalanceCommand. */
  public BalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new GalacPIDController2(kP, kI, kD, 0.01, () -> balanceSetpoint - Subsystems.driveSubsystem.getGyroPitch(), 0, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro pitch", Subsystems.driveSubsystem.getGyroPitch());
    SmartDashboard.putNumber("gyro yaw", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("gyro roll", Subsystems.driveSubsystem.getGyroRoll());
    SmartDashboard.putNumber("turning value", turningValue);
    SmartDashboard.putNumber("front right speed", Subsystems.driveSubsystem.frontRightLead.get());
    SmartDashboard.putNumber("front left speed", Subsystems.driveSubsystem.frontLeftLead.get());

    SmartDashboard.putNumber("balance effort", effort);

    // effort = (balanceSetpoint - Subsystems.driveSubsystem.getGyroRoll()) * kP;
    effort = pidController.getEffort();
    turningValue = (angleSetpoint - (Subsystems.driveSubsystem.getGyroAngle())) * kTurn;

    if(RobotContainer.driver.getRightBumper())
      Subsystems.driveSubsystem.drive(-effort, 0, -turningValue, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
