// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController2;

public class ForwardDistance extends CommandBase {

  double angleSetpoint = 0;
  double kP = 0.008;
  double turningValue;
  double m_distance;
  double m_speed;
  boolean isFieldOriented = false;
  
  
 GalacPIDController2  pidController;
  /** Creates a new AutonDriveCommand. */
  public ForwardDistance(double distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
  
    m_distance = distance;
    m_speed = speed;
    pidController = new GalacPIDController2(0.00201,0,0,0.005, () -> Subsystems.driveSubsystem.getAverageEncoderDistance(), m_distance, 1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.resetEncoders();
    Subsystems.driveSubsystem.setCoast();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;
    Subsystems.driveSubsystem.drive(-m_speed, 0, -turningValue, isFieldOriented);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.setBrake();
    Subsystems.driveSubsystem.drive(0, 0, 0, isFieldOriented);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Subsystems.driveSubsystem.getAverageEncoderDistance()) >= Math.abs(m_distance);
  }
}

