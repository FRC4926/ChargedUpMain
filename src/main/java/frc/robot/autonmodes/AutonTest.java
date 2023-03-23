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
    return new AutonArmCommand(false, 2)
    .andThen(new AutonIntakeCommand(1, -0.7)).alongWith(new AutonDriveCommand(10, -0.23))
    .andThen(new AutonDriveCommand(20, 0.3))
    .andThen(new TimedStrafe(-0.3, 0.35))
    .andThen(new AutonDriveCommand(120, 0.4).deadlineWith(new AutonArmCommand(false, 3)))
    .andThen(new AutonRotatewPID(183, 0.0027).deadlineWith(new AutonArmCommand(false, 0)))
    .andThen(new AutonIntakeCommand(1.1, 1).deadlineWith(new AutonTimedDrive(1.1, 0.18)))
    .andThen(new AutonRotatewPID(0, 0.004))
    .andThen(new AutonDriveCommand(210, -0.4).alongWith(
      new AutonArmCommand(false, 3).andThen(new WaitCommand(0.01)).andThen(new AutonArmCommand(false, 1))))
    .andThen(new AutonIntakeCommand(1, -0.5)).andThen(new AutonDriveCommand(25, 0.28).alongWith(new AutonArmCommand(false, 3)));
 }
}
