// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class AutonArmCommand extends CommandBase {
  /** Creates a new AutonArmCommand. */

  boolean object;
  int level;
  double targetElbowAngle;
  double targetShoulderAngle;
  double targetWristAngle;
  double targetGripPosition;
  //boolean {object} - true = cone, false = cube
  //int {level} - 0 = ground, 1 = middle, 2 = top
  public AutonArmCommand(boolean object, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.armSubsystem2);
    this.object = object;
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveElbow(0);
    Subsystems.armSubsystem2.moveWrist(0);
    Subsystems.armSubsystem2.moveGrip(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    switch (level) {
      case 0:
        Subsystems.armSubsystem2.moveToGround(object);
        break;
      
      case 1:
        if (object) {
          Subsystems.armSubsystem2.moveToLowerCone();
        } else {
          Subsystems.armSubsystem2.moveToLowerBox();

        }
        break;

      case 2:
        if (object) {
          Subsystems.armSubsystem2.moveToUpperCone();
        } else {
          Subsystems.armSubsystem2.moveToUpperBox();

        }
       break;

      default:
        break;
    }
    // if(Subsystems.armSubsystem2.shoulderMotor.getEncoder().getPosition()<=shoulderGearRatio*shoulderAngle/360)
    // {
    //   Subsystems.armSubsystem2.moveShoulder(shoulderEffort);
    // }
    // else
    // {
    //   Subsystems.armSubsystem2.moveShoulder(0);
    // }
    // if(Subsystems.armSubsystem2.elbowMotor.getEncoder().getPosition()<=elbowGearRatio*elbowAngle/360)
    // {
    //   Subsystems.armSubsystem2.moveElbow(elbowEffort);
    // }
    // else
    // {
    //   Subsystems.armSubsystem2.moveElbow(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveElbow(0);
    Subsystems.armSubsystem2.moveWrist(0);
    Subsystems.armSubsystem2.moveGrip(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.armSubsystem2.hasReachedTarget(object, level);
    //return (Subsystems.armSubsystem2.shoulderMotor.getEncoder().getPosition()>shoulderGearRatio*shoulderAngle/360) && (Subsystems.armSubsystem2.elbowMotor.getEncoder().getPosition()>elbowGearRatio*elbowAngle/360);
  }
}