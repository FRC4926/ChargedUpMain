// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class AutonRotate extends CommandBase {

  double angleSetpoint;
  double m_speed;

  
 GalacPIDController turnController;


  /** Creates a new AutonDriveCommand. */
  public AutonRotate(double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    angleSetpoint = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turnController = new GalacPIDController(0.0005, 0.001, 0, 0.005, () -> (Subsystems.driveSubsystem.getGyroAngle() % 360), angleSetpoint, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      Subsystems.driveSubsystem.drive(0, 0, m_speed, true);
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
    return Math.abs(Math.abs(Subsystems.driveSubsystem.getGyroAngle() % 360) - angleSetpoint) <= 1;
  }
}


