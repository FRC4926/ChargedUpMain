// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class AlignToZeroCommand extends CommandBase {

  double angleSetpoint = 0;
  double kP = 0.007;
  double turningValue;

  /** Creates a new AlignToZeroCommand. */
  public AlignToZeroCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("turning value", turningValue);

    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;

    if(RobotContainer.operator.getLeftBumper()){
      Subsystems.driveSubsystem.drive(0, 0, -turningValue, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
