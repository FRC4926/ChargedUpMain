// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController2;

public class VisionCommand extends CommandBase {
GalacPIDController2 pid;
double p=0.002;
double i=0;
double d=0;
double minEffort=0.007;
Supplier<Double> offset = () -> Subsystems.visionSubsystem.getHorizontalDistance()+0.0;
double positionTolerance = 2;
  /** Creates a new VisionCommand. */
  public VisionCommand() {
    //Cone/ball btw
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.visionSubsystem);
    // addRequirements(Subsystems.driveSubsystem);
    
  }

  

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    pid = new GalacPIDController2(p, i, d, minEffort, offset, 0, positionTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

       Subsystems.driveSubsystem.drive(-.152,0,-pid.getEffort(),true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.visionSubsystem.getDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.visionSubsystem.getDistance() < 35;
  }
}
