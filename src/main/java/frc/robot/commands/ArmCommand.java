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
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveForearm(0);
    Subsystems.armSubsystem2.resetEncoders();
    isAutomated = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pipelineNum = Subsystems.limelightSubsystem.getPipelineNum();

    if(RobotContainer.operator.getLeftStickButtonReleased()) 
      isAutomated = !isAutomated;

    SmartDashboard.putBoolean("isAutomated", isAutomated);
    SmartDashboard.putNumber("Shoulder Angle", Subsystems.armSubsystem2.getDegreesShoulder());
    SmartDashboard.putNumber("Forearm Angle", Subsystems.armSubsystem2.getDegreesForearm());




// move arm to upper, middle, or lower based on button click

    if(isAutomated)
    { 
      if(RobotContainer.operator.getYButton()){
          if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highCubeAngleForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highCubeAngleShoulder;
          }
          else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highConeAngleForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highConeAngleShoulder;
          }

          
        
      }

      else if(RobotContainer.operator.getBButton()){
        if(pipelineNum == 0){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowCubeAngleForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowCubeAngleShoulder;
        }
        else if(pipelineNum == 1){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowConeAngleForearm;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowConeAngleShoulder;
        }
      }

    //   else if(RobotContainer.operator.getAButton()){
    //     if(pipelineNum == 0){
    //       Subsystems.armSubsystem2.moveToGround();
    //     }
    //     else if(pipelineNum == 1){
    //       Subsystems.armSubsystem2.moveToGround();
    //     }
    //   }

    //   else if(RobotContainer.operator.getLeftBumper()){
    //     Subsystems.armSubsystem2.moveToReset();
    //   }

    //   else if(Math.abs(RobotContainer.operator.getPOV()) > 7.543)
    //   {
    //     SmartDashboard.putNumber("shoulder setpoint", Subsystems.armSubsystem2.pidControllerShoulder.getSetpoint());
    //     SmartDashboard.putNumber("forearm setpoint", Subsystems.armSubsystem2.pidControllerForearm.getSetpoint());
    //     Subsystems.armSubsystem2.holdSteady();
    //   }

        
    }
    else // control arm manually
    {
        if(Math.abs(RobotContainer.operator.getLeftY()) > 0.1){
          Subsystems.armSubsystem2.moveShoulder(RobotContainer.operator.getLeftY());
        } else {
          Subsystems.armSubsystem2.moveShoulder(0);
        }

        if(Math.abs(RobotContainer.operator.getRightY()) > 0.1){
          Subsystems.armSubsystem2.moveForearm(RobotContainer.operator.getRightY());
        } else {
          Subsystems.armSubsystem2.moveForearm(0);
        }

        if(RobotContainer.operator.getRightBumper()){
          Subsystems.armSubsystem2.moveGrip(0.3);
        }
        else if(RobotContainer.operator.getLeftBumper()){
          Subsystems.armSubsystem2.moveGrip(-0.3);
        }
        else
        Subsystems.armSubsystem2.moveGrip(0);



        if(Math.abs(RobotContainer.operator.getRightX()) > 0.1){
          if(RobotContainer.operator.getRightX() > 0.5){
            Subsystems.armSubsystem2.moveWrist(0.5);
          }else
            Subsystems.armSubsystem2.moveWrist(RobotContainer.operator.getRightX() - 0.1);
        } else {
          Subsystems.armSubsystem2.moveWrist(0);
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