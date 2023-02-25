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
  GalacPIDController2 balanceController;
  GalacPIDController2 turnController;

  double balanceSetpoint = 0;
  double kP = 0.00665;
  double kI = 0.00001;
  double kD = 0.00001;
  double balanceEffort;

  double modGyroYaw;

  double angleSetpoint = 0;
  double pTurn = 0.007;
  double iTurn = 0;
  double dTurn = 0;
  double turningEffort;


  /** Creates a new AutoBalanceCommand. */
  public BalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new GalacPIDController2(pTurn, iTurn, dTurn, 0.005, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), angleSetpoint, 1);
    balanceController = new GalacPIDController2(kP, kI, kD, 0.01, () -> Subsystems.driveSubsystem.getGyroPitch(), balanceSetpoint, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    modGyroYaw = Subsystems.driveSubsystem.getGyroAngle() % 360;    

    

   
    SmartDashboard.putNumber("gyro pitch", Subsystems.driveSubsystem.getGyroPitch());
    SmartDashboard.putNumber("modded gyro yaw", turnController.getMeasurementSupplier().get());
    SmartDashboard.putNumber("gyro yaw", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("gyro roll", Subsystems.driveSubsystem.getGyroRoll());
    SmartDashboard.putNumber("turning value", turningEffort);
    SmartDashboard.putNumber("balance effort", balanceEffort);

    // effort = (balanceSetpoint - Subsystems.driveSubsystem.getGyroRoll()) * kP;
    balanceEffort = balanceController.getEffort();
    if((modGyroYaw > 0 && modGyroYaw < 180) || (modGyroYaw < -180 && modGyroYaw > -360)){
      turningEffort = Math.abs(turnController.getEffort());
    }else{
      turningEffort = -Math.abs(turnController.getEffort());
    }

    if(RobotContainer.driver.getRightBumper())
      Subsystems.driveSubsystem.drive(-balanceEffort, 0, turningEffort, true);

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
