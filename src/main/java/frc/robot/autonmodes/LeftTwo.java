// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutoBalanceCommand;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonLimelightCommand;
import frc.robot.autoncommands.AutonStrafeCommand;
import frc.robot.autoncommands.AutonTimedStrafeCommand;
import frc.robot.autoncommands.MoveForwardCommand;
import frc.robot.autoncommands.TimedStrafe;
// import frc.robot.autoncommands.VisionCommand;

/** Add your docs here. */
public class LeftTwo {
    public static int pipelineNum;

    public LeftTwo() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
       Command m_autonomousCommand = (new AutonArmCommand(false, 2, false).andThen(new AutonDriveCommand(10, 0.2))
       .andThen(new AutonArmCommand(false, 2, true)).andThen(new AutonDriveCommand(200, -0.4))
       .deadlineWith(new AutonArmCommand(false, 3, false)).andThen(new AutonArmCommand(false, 4, false))
       .andThen(new AutonDriveCommand(180, 0.4)).andThen(new AutonArmCommand(false, 2, false))
       .andThen(new AutonLimelightCommand(true)).andThen(new AutonArmCommand(false, 2, true)));
       return m_autonomousCommand;
    }
}
