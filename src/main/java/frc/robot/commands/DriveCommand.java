// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final DriveSubsystem m_subsystem;

  
  public DriveCommand(){
   // m_subsystem = subsystem;                                                                                                                                                                                                        
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.setBrake();
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    double forward = RobotContainer.driver.getLeftY(); 
    double strafe = RobotContainer.driver.getLeftX();
    double rotate = RobotContainer.driver.getRightX();


    MathUtil.applyDeadband(forward, 0.02);
    MathUtil.applyDeadband(strafe, 0.02);
    MathUtil.applyDeadband(rotate, 0.02);
    
    Subsystems.driveSubsystem.drive(forward, -strafe, -rotate, true);
    
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
