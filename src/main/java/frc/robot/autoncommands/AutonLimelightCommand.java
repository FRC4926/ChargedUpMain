// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class AutonLimelightCommand extends CommandBase {

  double modGyroYaw;
  
  boolean isAprilTag;

  GalacPIDController forwardController;
  GalacPIDController strafeController;
  GalacPIDController turnController;

  /** Creates a new AutonLimelightCommand. */
  public AutonLimelightCommand(boolean target) {
    // Use addRequirements() here to declare subsystem dependencies.
    isAprilTag = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController = new GalacPIDController(0.013, 0, 0, 0.008, () -> (Subsystems.limelightSubsystem.getTY()), 1.13, 0.1);
    strafeController = new GalacPIDController(0.0145, 0, 0, 0.03, () -> (Subsystems.limelightSubsystem.getTX()), 11.5, 0.5);
    turnController = new GalacPIDController(0, 0, 0, 0.005, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), 0, 0.5);
    if(isAprilTag){
      Subsystems.limelightSubsystem.setPipeline(1);
      forwardController.setSetpoint(0);
    }
    else{
      Subsystems.limelightSubsystem.setPipeline(0);
      forwardController.setSetpoint(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.driveSubsystem.drive(0, -strafeController.getEffort(), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(Subsystems.limelightSubsystem.getTX()) < 0.5;
  }
}
