// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable.SubTableListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(RobotContainer.operator.getLeftX()) < 0.05 || Math.abs(RobotContainer.operator.getRightX()) < 0.05){
      Subsystems.armSubsystem.moveShoulder(0);
      Subsystems.armSubsystem.moveElbow(0);
    }
    if(Math.abs(RobotContainer.operator.getLeftX()) >= 0.05){
      Subsystems.armSubsystem.moveShoulder(RobotContainer.operator.getLeftX());
    }

    if(Math.abs(RobotContainer.operator.getRightX()) >= 0.05){
      Subsystems.armSubsystem.moveElbow(RobotContainer.operator.getRightX());
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
