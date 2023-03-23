// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class AutonTimedDrive extends CommandBase {

  Timer timer;
  double m_time;
  double m_speed;

  /** Creates a new AutonIntakeCommand. */
  public AutonTimedDrive(double time, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_time = time;
    m_speed = speed;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.driveSubsystem.drive(-m_speed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= m_time;
  }

}
