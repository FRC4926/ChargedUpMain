// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class AutonDriveCommand2 extends CommandBase {

  double angleSetpoint;
  double m_distance;
  double m_speed;
  double turningEffort;

  double modGyroYaw;
  
  
 GalacPIDController forwardController;
 GalacPIDController turnController;


  /** Creates a new AutonDriveCommand. */
  public AutonDriveCommand2(double distance, double speed, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    angleSetpoint = angle;
    m_distance = distance;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.resetEncoders();
    Subsystems.driveSubsystem.setCoast();
    turnController = new GalacPIDController(0.002, 0, 0, 0.005, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), angleSetpoint, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    modGyroYaw = Subsystems.driveSubsystem.getGyroAngle() % 360;    
    if((modGyroYaw > 0 && modGyroYaw < 180) || (modGyroYaw < -180 && modGyroYaw > -360)){
      turningEffort = Math.abs(turnController.getEffort());
    }else{
      turningEffort = -Math.abs(turnController.getEffort());
    }
    Subsystems.driveSubsystem.drive(-m_speed, 0, turningEffort, true);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.setBrake();
    Subsystems.driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Subsystems.driveSubsystem.getAverageEncoderDistance()) >= Math.abs(m_distance) && Math.abs(modGyroYaw) < Math.abs(angleSetpoint);
  }
}

