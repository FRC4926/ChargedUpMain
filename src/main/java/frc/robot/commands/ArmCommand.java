// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  boolean isAutomated;
  int pipelineNum;
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem.moveShoulder(0);
    Subsystems.armSubsystem.moveForearm(0);
    Subsystems.armSubsystem.moveWrist(0);
    Subsystems.armSubsystem.resetEncoders();
    isAutomated = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pipelineNum = Subsystems.limelightSubsystem.getPipelineNum();

    if(RobotContainer.operator2.getLeftStickButtonReleased()) 
      isAutomated = !isAutomated;

    SmartDashboard.putBoolean("isAutomated", isAutomated);
    SmartDashboard.putBoolean("button test", RobotContainer.operator.getRawButton(11));

    SmartDashboard.putNumber("Shoulder Angle", Subsystems.armSubsystem.getDegreesShoulder());
    SmartDashboard.putNumber("Forearm Angle", Subsystems.armSubsystem.getDegreesForearm());


    if(isAutomated)
    { 
      if(RobotContainer.operator.getRawButton(7)){
          if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highCubeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highCubeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.highCubeWrist;
          }
          else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highConeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highConeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.highConeWrist;
          }
      }

      else if(RobotContainer.operator.getRawButton(9)){
        if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowCubeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowCubeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.lowCubeWrist;
        }
        else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowConeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowConeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.lowConeWrist;
        }
      }

      else if(RobotContainer.operator.getRawButton(11)){
        if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.floorCubeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.floorCubeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.floorCubeWrist;
        }
        else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.floorConeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.floorConeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.floorConeWrist;
        }
      }

      else if(RobotContainer.operator.getRawButton(2)){
        if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.substationCubeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.substationCubeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.substationCubeWrist;
        }
        else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.substationConeForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.substationConeShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.substationConeWrist;
        }
      }

      else if(RobotContainer.operator.getRawButton(1)){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.resetForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.resetShoulder;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.resetWrist;
        
      }


        
    }
    else // control arm manually
    {
        if(Math.abs(RobotContainer.operator.getRawAxis(0)) > 0.02){
          Subsystems.armSubsystem.moveIntake(RobotContainer.operator.getRawAxis(0));
        }
        else{
          Subsystems.armSubsystem.moveIntake(0);
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