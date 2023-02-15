// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class AutonStrafeCommand extends CommandBase {

  double angleSetpoint = 0;
  double kP = 0.014;
  double strafeP = 0.01;
  double turningValue;
  double tx;
  double ty;
  double strafeLimelight;
  double strafe;
  int pipelineNum;
  /** Creates a new AutonLimelightCommand. */
  public AutonStrafeCommand(double strafeSpeed, int pipelineNumber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.limelightSubsystem);
    addRequirements(Subsystems.driveSubsystem);
    strafe = strafeSpeed;
    pipelineNum = pipelineNumber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.setCoast();
    Subsystems.limelightSubsystem.setPipeline(pipelineNum);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    tx = Subsystems.limelightSubsystem.getTX();
    ty = Subsystems.limelightSubsystem.getTY();
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;
    strafeLimelight = (-tx * strafeP);
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("turning value", turningValue);
    SmartDashboard.putNumber("strafe value", strafeLimelight);

    if(!Subsystems.limelightSubsystem.getTV()){
      SmartDashboard.putString("target", "cant see the target");

      Subsystems.driveSubsystem.drive(0, -strafe, -turningValue, true);
    }
    else if(Subsystems.limelightSubsystem.getTV()){
      SmartDashboard.putString("target", "can see the target");

      Subsystems.driveSubsystem.drive(0, -strafeLimelight, -turningValue, true);
    }

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
    return Math.abs(tx) < 2 && Subsystems.limelightSubsystem.getTV();
  }
}