// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonRotate;
import frc.robot.autoncommands.AutonRotatewPID;
import frc.robot.autoncommands.AutonTimedDrive;
import frc.robot.autoncommands.TimedStrafe;
import frc.robot.autoncommands.AutonIntakeCommand;
import frc.robot.autoncommands.AutonLimelightCommand;

/** Add your docs here. */
public class AutonTest {

 public AutonTest() {

 }   

 public static Command getCommand() {
  return new AutonArmCommand(false, 2).andThen(new AutonIntakeCommand(0.7, 1))
  .andThen(new AutonArmCommand(false, 3).deadlineWith(new AutonDriveCommand(5, 0.1)))
  .andThen(new TimedStrafe(0.3, 0.45))
  .andThen(new AutonDriveCommand(132, 0.3))
  .andThen(new TimedStrafe(0.3, 0.25))
  .andThen(new AutonRotatewPID(180, 0.0027))
  .andThen(new AutonArmCommand(false, 0))
  .andThen(new AutonIntakeCommand(3, 1).deadlineWith(new AutonTimedDrive(0.75, 0.2)))
  .andThen(new AutonArmCommand(false, 3))
  .andThen(new AutonRotatewPID(-10, 0.0062))
  .andThen(new AutonDriveCommand(160, -0.3)).andThen(new AutonArmCommand(false, 2))
  .andThen(new AutonIntakeCommand(0.7, -0.3))
  .andThen(new AutonArmCommand(false, 3).alongWith(new AutonDriveCommand(20, 0.3)));
 }
}
