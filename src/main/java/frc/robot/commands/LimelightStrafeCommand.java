// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class LimelightStrafeCommand extends CommandBase {

  // variables to align yaw to zero
  double angleSetpoint = 180;
  double kP = 0.005;
  double turningValue;

  // variables to strafe to center of target
  double strafeP = 0.015;
  double strafingValue;
  double minEffort = 0.05;
  double tx;
  double ty;

  boolean isAprilTag = false;

  /** Creates a new LimelightStrafeCommand. */
  public LimelightStrafeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Subsystems.limelightSubsystem);
    // addRequirements(Subsystems.driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = Subsystems.limelightSubsystem.getTX();
    ty = Subsystems.limelightSubsystem.getTY();
    turningValue = (angleSetpoint - Subsystems.driveSubsystem.getGyroAngle()) * kP;
    strafingValue = (-tx * strafeP);
    SmartDashboard.putNumber("limelight tx", tx);
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());
    SmartDashboard.putNumber("limelight ty", ty);
    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());

    // toggle limelight pipeline with pressing the 'X Button'. 
    // true = april tag |||| false = reflective tape
    if(RobotContainer.operator.getXButtonReleased()){
      isAprilTag = !isAprilTag;
      if(isAprilTag){
        Subsystems.limelightSubsystem.setPipeline(0);
      }
      else{
        Subsystems.limelightSubsystem.setPipeline(1);
      }
    }
    // if target is centered enough, move forward until ty reaches a certain value
    // AKA a good place to stop
    // if(RobotContainer.driver.getAButton() && Math.abs(Subsystems.limelightSubsystem.getTX() + 23) < 2 && Subsystems.limelightSubsystem.getTY() > -8){
    //   Subsystems.driveSubsystem.drive(0.2, 0, -turningValue, true);
    // }
    // strafe to center of target
    // only moves robot when target is in view (tv = 1)    
    if(RobotContainer.driver.getAButton()){
      if(Math.abs(Subsystems.limelightSubsystem.getTX()+20)>2 && Math.abs(Math.abs(Subsystems.driveSubsystem.getGyroAngle()%360)-angleSetpoint)<5)
        {
          Subsystems.driveSubsystem.drive(0, strafingValue, 0, true);
          SmartDashboard.putString("pathway", "strafe");
        }
      else if(Math.abs(Subsystems.limelightSubsystem.getTX()+20)<2 && Math.abs(Math.abs(Subsystems.driveSubsystem.getGyroAngle()%360)-angleSetpoint)<2)
        {
          Subsystems.driveSubsystem.drive(0.2, 0, 0, true);
          SmartDashboard.putString("pathway", "forward");
        }
      else
        {

          Subsystems.driveSubsystem.drive(0, 0, -turningValue, true);
          SmartDashboard.putString("pathway", "turn");
        }
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
