// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class LimelightStrafeCommand extends CommandBase {

  double modGyroYaw;
  
  boolean isAprilTag = false;

  double turningEffort;

  GalacPIDController forwardController;
  GalacPIDController strafeController;
  GalacPIDController turnController;



  /** Creates a new LimelightStrafeCommand2. */
  public LimelightStrafeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      forwardController = new GalacPIDController(0.02, 0, 0, 0.01, () -> (Subsystems.limelightSubsystem.getTY()), 1.13, 0.1);
      strafeController = new GalacPIDController(0.016, 0, 0, 0.01, () -> (Subsystems.limelightSubsystem.getTX()), 12, 0.5);
      turnController = new GalacPIDController(0.005, 0, 0, 0.01, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), 0, 0.5);

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("gyro pitch", Subsystems.driveSubsystem.getGyroPitch());
    modGyroYaw = Subsystems.driveSubsystem.getGyroAngle() % 360;
    if(RobotContainer.driver.getLeftBumperReleased
    ()){
      isAprilTag = !isAprilTag;
      if(!isAprilTag){
        Subsystems.limelightSubsystem.setPipeline(0);
        forwardController.setSetpoint(1.13);
      }
      else if(isAprilTag){
        Subsystems.limelightSubsystem.setPipeline(1);
        forwardController.setSetpoint(0);
      }
    }

    if((modGyroYaw > 0 && modGyroYaw < 180) || (modGyroYaw < -180 && modGyroYaw > -360)){
      turningEffort = Math.abs(turnController.getEffort());
    }else{

      turningEffort = -Math.abs(turnController.getEffort());
    }

    if(RobotContainer.driver.getAButton() && Subsystems.limelightSubsystem.getTV()){
      Subsystems.driveSubsystem.drive(0, -strafeController.getEffort(), turningEffort, true);
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
