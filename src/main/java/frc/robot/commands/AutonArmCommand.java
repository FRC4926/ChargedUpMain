// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;

public class AutonArmCommand extends CommandBase {
  /** Creates a new AutonArmCommand. */

  boolean object;
  int level;
  double targetElbowAngle;
  double targetShoulderAngle;
  double targetWristAngle;
  double targetGripPosition;
  boolean released =false;

  // boolean {object} - true = cone, false = cube
  // int {level} - 0 = ground, 1 = middle, 2 = top
  // boolean {relase} - true if want to release object otherwise false
  public AutonArmCommand(boolean object, int level, boolean release) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.armSubsystem2);
    this.object = object;
    this.level = level;
    this.released = release;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveForearm(0);
    Subsystems.armSubsystem2.moveGrip(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("hasReachedTarget", Subsystems.armSubsystem2.hasReachedTarget());
    SmartDashboard.putNumber("auton forearm angle", Subsystems.armSubsystem2.getDegreesForearm());
    SmartDashboard.putNumber("auton shoulder angle", Subsystems.armSubsystem2.getDegreesShoulder());
    Subsystems.armSubsystem2.displayAngles();

    if(released){
      Subsystems.armSubsystem2.release();
    }
    else{
    switch (level) {
      case 0:
        Subsystems.armSubsystem2.moveToGround();
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
      
      case 3:
        Subsystems.armSubsystem2.moveToReset();
        break;

      default:
        break;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.armSubsystem2.moveShoulder(0);
    Subsystems.armSubsystem2.moveForearm(0);
    Subsystems.armSubsystem2.moveGrip(0);
    Subsystems.armSubsystem2.movePad(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.armSubsystem2.hasReachedTarget();
  }
}