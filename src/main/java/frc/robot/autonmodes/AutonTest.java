// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonDriveCommand2;
import frc.robot.autoncommands.AutonIntakeCommand;
import frc.robot.autoncommands.AutonLimelightCommand;

/** Add your docs here. */
public class AutonTest {

 public AutonTest() {

 }   

 public static Command getCommand() {
    return new AutonDriveCommand2(15, 0.2, 180).alongWith(new AutonArmCommand(false, 2)).andThen(new AutonIntakeCommand(1.3, 0.6));
 }
}
