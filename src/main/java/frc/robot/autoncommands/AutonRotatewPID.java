// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class AutonRotatewPID extends CommandBase {
  //Robot is balanced at 0 degrees at charge station
  double balanceSetpoint = 0;
  //Proportion value for PID
  double kP = 0.008;
  //Effort to apply to the motor
  double balanceEffort;
  // Robot aligns to 0 degrees 
  double angleSetpoint = 0;
  //Proportion constant for turning
  double kTurn;
  // turning effort
  double turningEffort;
  /** Creates a new AutoBalanceCommand. */
  public AutonRotatewPID(double angle, double p){
    // Use addRequirements() here to declare subsystem dependencies.
    angleSetpoint = angle;
    kTurn = p;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Subsystems.armSubsystem2.holdSteady();
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());

    
    turningEffort = 0.024 + (angleSetpoint - (Subsystems.driveSubsystem.getGyroAngle())) * kTurn;

    Subsystems.driveSubsystem.drive(0, 0, -turningEffort, true);
    

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
    return Math.abs((Subsystems.driveSubsystem.getGyroAngle()) - angleSetpoint) < 2;
  }
}
