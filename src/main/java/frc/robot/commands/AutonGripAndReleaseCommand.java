// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;

public class AutonGripAndReleaseCommand extends CommandBase {
  /** Creates a new AutonGripAndReleaseCommand. */
  boolean grip;
  boolean cone;
  public AutonGripAndReleaseCommand(boolean grip, boolean cone) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.armSubsystem2);
    grip = this.grip;
    cone = this.cone;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(grip){
      if(cone)
      Subsystems.armSubsystem2.grabCone();
      else
      Subsystems.armSubsystem2.grabCube();
    }
    else{
      Subsystems.armSubsystem2.release();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.armSubsystem2.pidControllerGrip.isFinished();
  }
}
