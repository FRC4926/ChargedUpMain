// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(RobotContainer.driver.getRightTriggerAxis() > 0.02)
      Subsystems.intakeSubsystem.runIntake(RobotContainer.driver.getRightTriggerAxis());
    
    else if(RobotContainer.driver.getLeftTriggerAxis() > 0.02){
      Subsystems.intakeSubsystem.runIntake(-RobotContainer.driver.getLeftTriggerAxis());
    }
    else{
      Subsystems.intakeSubsystem.runIntake(0);
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
