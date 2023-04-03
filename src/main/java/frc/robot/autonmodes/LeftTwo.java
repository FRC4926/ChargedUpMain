// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonIntakeCommand;
import frc.robot.autoncommands.AutonLimelightCommand;
import frc.robot.autoncommands.AutonRotatewPID;
import frc.robot.autoncommands.AutonTimedDrive;
import frc.robot.autoncommands.TimedStrafe;

/** Add your docs here. */
public class LeftTwo {
    public static int pipelineNum;

    public LeftTwo() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
        return new AutonArmCommand(false, 2)
        .andThen(new AutonIntakeCommand(1, -0.7))
        .andThen(new TimedStrafe(0.3, 0.35).alongWith(new AutonArmCommand(false, 3)))
        .andThen(new AutonDriveCommand(120, 0.4))
        .andThen(new AutonRotatewPID(-183, 0.0027)).andThen(new AutonArmCommand(false, 0))
        .andThen(new AutonIntakeCommand(1.1, 1).deadlineWith(new AutonTimedDrive(1.1, 0.18)))
        .andThen(new AutonRotatewPID(0, 0.004))
        .andThen(new AutonDriveCommand(210, -0.4).alongWith(
          new AutonArmCommand(false, 3).andThen(new WaitCommand(0.01)).andThen(new AutonArmCommand(false, 1))))
        .andThen(new AutonIntakeCommand(1, -0.5)).andThen(new AutonDriveCommand(25, 0.28).alongWith(new AutonArmCommand(false, 3)));
    }
}
