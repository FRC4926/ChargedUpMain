// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;

public class AutonTimedStrafeCommand extends CommandBase {
  /** Creates a new AutonTimedStrafeCommand. */
  Timer timer = new Timer();
  double effort;
  double time;

  double turningValue;
  double angleSetpoint = 0;
  double kP = 0.01;

    public AutonTimedStrafeCommand(double m_effort, double m_time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
    effort = m_effort;
    time = m_time;
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
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;

    Subsystems.driveSubsystem.drive(0, effort, -turningValue, true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
