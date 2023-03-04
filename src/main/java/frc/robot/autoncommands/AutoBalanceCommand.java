// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class AutoBalanceCommand extends CommandBase {
  //Robot is balanced at 0 degrees at charge station
  double balanceSetpoint = 0;
  //Proportion value for PID
  double kP = 0.01;
  //Effort to apply to the motor
  double balanceEffort;
  // Robot aligns to 0 degrees 
  double angleSetpoint = 0;
  //Proportion constant for turning
  double kTurn = 0.007;
  // turning effort
  double turningEffort;
  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro auton roll", Subsystems.driveSubsystem.getGyroRoll());
    SmartDashboard.putNumber("turning value", balanceEffort);

    // Subsystems.armSubsystem2.holdSteady();
    
    turningEffort = (angleSetpoint - (Subsystems.driveSubsystem.getGyroAngle()%360)) * kTurn;
    balanceEffort = (balanceSetpoint - Subsystems.driveSubsystem.getGyroPitch()) * kP;

    Subsystems.driveSubsystem.drive(balanceEffort, 0, -turningEffort, true);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.drive(0, 0, 0, true);
    Subsystems.driveSubsystem.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(Subsystems.driveSubsystem.getGyroPitch()) < 2;
    return false;
  }
}
