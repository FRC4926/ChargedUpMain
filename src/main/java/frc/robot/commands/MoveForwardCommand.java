// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class MoveForwardCommand extends CommandBase {
  double ty;
  int pipelineNum;
  double turningValue;
  double angleSetpoint = 0;
  double kP = 0.01;
  /** Creates a new MoveForwardCommand. */
  public MoveForwardCommand(int pipelineNumber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.limelightSubsystem);
    addRequirements(Subsystems.driveSubsystem);
    pipelineNum = pipelineNumber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.limelightSubsystem.setPipeline(pipelineNum);
    Subsystems.driveSubsystem.setCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = Subsystems.limelightSubsystem.getTY();
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;


    if(pipelineNum == 0){
      if(Subsystems.limelightSubsystem.getTY() > -12){
        Subsystems.driveSubsystem.drive(0.2, 0, -turningValue, true);
      }
    }
    else if(pipelineNum == 1){
      if(Subsystems.limelightSubsystem.getTY() > -5){
        Subsystems.driveSubsystem.drive(0.2, 0, -turningValue, true);

      }
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
    if(pipelineNum == 0){
      return Math.abs(Subsystems.limelightSubsystem.getTY() + 12) < 0.25;
    }
    else{
      return Math.abs(Subsystems.limelightSubsystem.getTY() + 5) < 0.5;

    }

  }
}
