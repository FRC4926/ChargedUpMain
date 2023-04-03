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
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem.moveShoulder(0);
    Subsystems.armSubsystem.moveForearm(0);
    Subsystems.armSubsystem.moveWrist(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.operator.getRawButton(6)){
      RobotContainer.isAutomated = !RobotContainer.isAutomated;
    }

    if(RobotContainer.isAutomated){
      
      if(RobotContainer.operator.getRawButton(7)){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highForearm;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.highWrist;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highShoulder;
      }

      else if(RobotContainer.operator.getRawButton(9)){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowForearm;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.lowWrist;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowShoulder;

      }

      else if(RobotContainer.operator.getRawButton(11)){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.floorForearmCone;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.floorWristCone;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.floorShoulderCone;
      }

      else if(RobotContainer.operator.getRawButton(12)){
        Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.floorForearmCube;
        Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.floorWristCube;
        Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.floorShoulderCube;
      }

      else if(RobotContainer.operator.getRawButton(2)){
            Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.substationForearm;
            Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.substationWrist;
            Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.substationShoulder;
      }

      else if(RobotContainer.operator.getRawButton(1)){
        Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.singleSubForearm;
        Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.singleSubShoulder;
        Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.singleSubWrist;
      }      
      else{
        Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.resetForearm;
        Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.resetWrist;
        Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.resetShoulder;
      }

      if(RobotContainer.operator.getRawButton(10)){
        // Subsystems.armSubsystem.forearmState += 5;
        Subsystems.armSubsystem.wristState += 15;
      }

      if(RobotContainer.operator.getRawButton(8)){
        // Subsystems.armSubsystem.forearmState -= 5;
        Subsystems.armSubsystem.wristState -= 15;
      }      

    

      if(Math.abs(RobotContainer.operator.getRawAxis(1)) > 0.2){
        Subsystems.armSubsystem.moveIntake(RobotContainer.operator.getRawAxis(1));
      }
      else{
        Subsystems.armSubsystem.moveIntake(0);

      }
      
    } // end isAutomated

    else{
      if(Math.abs(RobotContainer.operator.getRawAxis(3)) > 0.6){
        Subsystems.armSubsystem.moveIntake(RobotContainer.operator.getRawAxis(3));
      }
      else{
        Subsystems.armSubsystem.moveIntake(0);
      }

      if(Math.abs(RobotContainer.operator.getRawAxis(0)) >= 0.3){
        Subsystems.armSubsystem.moveForearm(RobotContainer.operator.getRawAxis(0));
      }
      else{
        Subsystems.armSubsystem.moveForearm(0);
      }

      if(Math.abs(RobotContainer.operator.getRawAxis(2)) > 0.3){
        Subsystems.armSubsystem.moveWrist(RobotContainer.operator.getRawAxis(2));
      }
      else{
        Subsystems.armSubsystem.moveWrist(0);
      }

      if(Math.abs(RobotContainer.operator.getRawAxis(1)) > 0.3){
        Subsystems.armSubsystem.moveShoulder(RobotContainer.operator.getRawAxis(1));
      }
      else{
        Subsystems.armSubsystem.moveShoulder(0);
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