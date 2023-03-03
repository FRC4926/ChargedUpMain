// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.GalacPIDController2;

public class AutonNavigation extends CommandBase {
  GalacPIDController2 pid;
  double p=0.002;
  double i=0;
  double d=0;
  double minEffort=0.007;
  Supplier<Double> offset = () -> Subsystems.visionSubsystem.getHorizontalDistance()+0.0;
  double positionTolerance = 2;
  /** Creates a new AutonNavigation. */
  public AutonNavigation() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.driveSubsystem.drive(.15, 0, pid.getEffort(), true);
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
