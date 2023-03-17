// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;

public class AutonIntakeCommand extends CommandBase {

  Timer timer;
  double time;
  double speed;

  /** Creates a new AutonIntakeCommand. */
  public AutonIntakeCommand(double time, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    time = this.time;
    speed = this.speed;
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
    Subsystems.armSubsystem.moveIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.armSubsystem.moveIntake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
