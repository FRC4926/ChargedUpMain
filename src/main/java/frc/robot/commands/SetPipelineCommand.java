// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class SetPipelineCommand extends CommandBase {
  /** Creates a new SetPipelineCommand. */
  public SetPipelineCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.operator2.getRawButton(1)){
      Subsystems.limelightSubsystem.setPipeline(0);
      SmartDashboard.putNumber("pipeline number", 0);
    }else if(RobotContainer.operator2.getRawButton(11)){
      Subsystems.limelightSubsystem.setPipeline(1);
      SmartDashboard.putNumber("pipeline number", 1);
    }else if(RobotContainer.operator2.getRawButton(9)){
      Subsystems.limelightSubsystem.setPipeline(2);
      SmartDashboard.putNumber("pipeline number", 2);
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
