// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class LimelightStrafeCommand extends CommandBase {

  double angleSetpoint = 0;
  double kP = 0.01;
  double strafeP = 0.015;
  double turningValue;
  double strafingValue;
  double minEffort = 0.05;
  double tx;
  double ty;

  /** Creates a new LimelightStrafeCommand. */
  public LimelightStrafeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.limelightSubsystem);
    // addRequirements(Subsystems.driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = Subsystems.limelightSubsystem.getTX();
    ty = Subsystems.limelightSubsystem.getTY();
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;
    strafingValue = (-tx * strafeP);
    SmartDashboard.putNumber("limelight tx", tx);
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("limelight ty", ty);

    if(RobotContainer.operator.getXButton()){
      Subsystems.limelightSubsystem.setPipeline(0);
      SmartDashboard.putNumber("pipeline number", 0);
    }else if(RobotContainer.operator.getBButton()){
      Subsystems.limelightSubsystem.setPipeline(1);
      SmartDashboard.putNumber("pipeline number", 1);    
    }
    
    if(RobotContainer.operator.getAButton() && Subsystems.limelightSubsystem.getTV()){
      Subsystems.driveSubsystem.drive(0, -strafingValue, -turningValue, true);
    }
    if(RobotContainer.operator.getAButton() && Math.abs(Subsystems.limelightSubsystem.getTX()) < 2 && Subsystems.limelightSubsystem.getTY() > -8){
      Subsystems.driveSubsystem.drive(0.2, 0, -turningValue, true);
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