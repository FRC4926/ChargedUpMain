// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;

public class AutonArmCommand extends CommandBase {
  /** Creates a new AutonArmCommand. */
  boolean object;
  int level;
  Timer timer;

  // boolean {object} - true = cone, false = cube
  // int {level} - 0 = ground, 1 = middle, 2 = top
  // boolean {relase} - true if want to release object otherwise false
  public AutonArmCommand(boolean object, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.armSubsystem);
    this.object = object;
    this.level = level;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Subsystems.armSubsystem.setBrakeArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    switch (level) {
      case 0:
        Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.floorForearmCube;
        Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.floorWristCube + 1.5;
        Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.floorShoulderCube;

    break;

      case 1:
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.lowForearm;
          Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.lowWrist;
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.lowShoulder;

        break;

      case 2:
          Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.highShoulder;
          Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.highForearm;
          Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.highWrist;
        break;
      
      case 3:
        Subsystems.armSubsystem.forearmState = Constants.ArmSetpoints.resetForearm;
        Subsystems.armSubsystem.wristState = Constants.ArmSetpoints.resetWrist;
        Subsystems.armSubsystem.shoulderState = Constants.ArmSetpoints.resetShoulder;

  break;

      default:
        break;
    
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 1.8;
  }
}