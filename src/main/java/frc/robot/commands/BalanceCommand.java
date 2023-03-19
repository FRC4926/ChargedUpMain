// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class BalanceCommand extends CommandBase {
  GalacPIDController balanceController;
  GalacPIDController turnController;

  double balanceSetpoint = 0;
  double kP = 0.0065;
  double kI = 0.0003;
  double kD = 0.0000;
  double balanceEffort;

  double modGyroYaw;

  double angleSetpoint = 0;
  double pTurn = 0.004;
  double iTurn = 0;
  double dTurn = 0;
  double turningEffort;


  /** Creates a new AutoBalanceCommand. */
  public BalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new GalacPIDController(pTurn, iTurn, dTurn, 0.005, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), angleSetpoint, 0);
    balanceController = new GalacPIDController(kP, kI, kD, 0.01, () -> Subsystems.driveSubsystem.getGyroPitch(), balanceSetpoint, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    modGyroYaw = Subsystems.driveSubsystem.getGyroAngle() % 360;    

    balanceEffort = balanceController.getEffort();
    if((modGyroYaw > 0 && modGyroYaw < 180) || (modGyroYaw < -180 && modGyroYaw > -360)){
      turningEffort = Math.abs(turnController.getEffort());
    }else{
      turningEffort = -Math.abs(turnController.getEffort());
    }

    if(RobotContainer.driver.getRightBumper())
      Subsystems.driveSubsystem.drive(balanceEffort, 0, turningEffort, true);

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
