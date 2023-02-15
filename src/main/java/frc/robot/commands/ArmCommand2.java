// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class ArmCommand2 extends CommandBase {
  /** Creates a new ArmCommand. */
  boolean isAutomated;
  public ArmCommand2() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Subsystems.armSubsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveElbow(0);
    Subsystems.armSubsystem2.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.getYButtonPressed())
      isAutomated = !isAutomated;
    SmartDashboard.putNumber("Shoulder controller input", RobotContainer.operator.getLeftX());
    SmartDashboard.putNumber("Elbow controller input", RobotContainer.operator.getRightX());
    SmartDashboard.putNumber("Shoulder Angle", Subsystems.armSubsystem2.shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Elbow Angle", Subsystems.armSubsystem2.elbowEncoder.getPosition());
    
    // if(isAutomated)
    // {
        
    // }
    // else
    // {
      if(Math.abs(RobotContainer.operator.getLeftX()) < 0.05){
        Subsystems.armSubsystem2.moveShoulder(0);
      }
      if(Math.abs(RobotContainer.operator.getRightX()) < 0.05){
        Subsystems.armSubsystem2.moveElbow(0);
      }
      if(Math.abs(RobotContainer.operator.getLeftX()) >= 0.05){
        Subsystems.armSubsystem2.moveShoulder(RobotContainer.operator.getLeftX());
      }

      if(Math.abs(RobotContainer.operator.getRightX()) >= 0.05){
        Subsystems.armSubsystem2.moveElbow(RobotContainer.operator.getRightX());

      // }

      // speed value not tested
      // not sure which way positive or negative
      if (RobotContainer.operator.getRightBumper()) {
        Subsystems.armSubsystem2.gripMotor.set(0.1);
      }

      if (RobotContainer.operator.getLeftBumper()) {
        Subsystems.armSubsystem2.gripMotor.set(-0.1);
      }
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