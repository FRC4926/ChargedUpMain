// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;

public class AutonArmCommand extends CommandBase {
  /** Creates a new AutonArmCommand. */
  boolean object;
  int level;
  boolean released;

  // boolean {object} - true = cone, false = cube
  // int {level} - 0 = ground, 1 = middle, 2 = top
  // boolean {relase} - true if want to release object otherwise false
  public AutonArmCommand(boolean object, int level, boolean release) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.armSubsystem);
    this.object = object;
    this.level = level;
    this.released = release;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem.moveShoulder(0);
    Subsystems.armSubsystem.moveForearm(0);
    Subsystems.armSubsystem.moveGrip(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("auton forearm angle", Subsystems.armSubsystem.getDegreesForearm());
    SmartDashboard.putNumber("auton shoulder angle", Subsystems.armSubsystem.getDegreesShoulder());

    // if(released){
    //   Subsystems.armSubsystem.release();
    // }
    
    switch (level) {
      case 0:
      // set forearm and shoulder setpoints to ground level
        break;

      case 1:
        if (object) {
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowConeAngleForearm;
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowConeAngleShoulder;
        } else {
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowCubeAngleForearm;
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowCubeAngleShoulder;
        }
        break;

      case 2:
        if (object) {
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highConeAngleForearm;
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highConeAngleShoulder;
        } else {
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highCubeAngleForearm;
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highCubeAngleShoulder;

        }
        break;
      
      case 3:
      // set forearm and shoulder setpoints to reset level
      break;

      default:
        break;
    
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.armSubsystem.moveShoulder(0);
    Subsystems.armSubsystem.moveForearm(0);
    Subsystems.armSubsystem.moveGrip(0);
    Subsystems.armSubsystem.moveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.armSubsystem.hasReachedTarget();
  }
}