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
import frc.robot.autoncommands.AutonRotate;
import frc.robot.autoncommands.AutonRotatewPID;
import frc.robot.autoncommands.AutonTimedDrive;
import frc.robot.autoncommands.TimedStrafe;
// import frc.robot.autoncommands.VisionCommand;


/** Add your docs here. */
public class RightTwo {
    public static int pipelineNum;

    public RightTwo() {
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
        return new AutonArmCommand(false, 2).andThen(new AutonIntakeCommand(0.45, 1))
  .andThen(new AutonDriveCommand(5, 0.1))
  .andThen(new AutonArmCommand(false, 3).alongWith(new TimedStrafe(0.3, 0.35).andThen(new AutonDriveCommand(132, 0.4))))
  .andThen(new AutonRotatewPID(180, 0.0027).deadlineWith(new WaitCommand(0.25).andThen(new AutonArmCommand(false, 0))))
  .andThen(new AutonIntakeCommand(1.5, 1).deadlineWith(new AutonTimedDrive(0.75, 0.2)))
  .andThen(new AutonRotatewPID(-12, 0.0062).deadlineWith(new AutonArmCommand(false, 3)))
  .andThen(new AutonDriveCommand(175, -0.28).alongWith(new WaitCommand(1.7).andThen(new AutonArmCommand(false, 2))))
  .andThen(new AutonIntakeCommand(0.7, -0.6))
  .andThen(new TimedStrafe(-0.3, 0.2))
  .andThen(new AutonArmCommand(false, 3).alongWith(new AutonDriveCommand(175, 0.4)))
  .andThen(new AutonRotatewPID(130, 0.003));
    }
}
