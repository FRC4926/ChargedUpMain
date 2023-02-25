// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutonArmCommand;
import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.ForwardDistance;
import frc.robot.commands.AutonStrafeCommand;
import frc.robot.commands.AutonTimedStrafeCommand;
import frc.robot.commands.TimedStrafe;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.MoveForwardCommand;

/** Add your docs here. */
public class AutonTesting {
    public static int pipelineNum;

    public AutonTesting() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
        
        Command m_autonomousCommand = (new AutonStrafeCommand(-0.1, pipelineNum).andThen(new AutonArmCommand(false, 2, false))
        .andThen(new MoveForwardCommand(pipelineNum)).andThen(new AutonDriveCommand(25, -0.1)));
        // Command m_autonomousCommand = (new AutonArmCommand(false, 2, false).andThen(new AutonArmCommand(false, 3, false))
        // .andThen(new AutonDriveCommand(35.45, -0.1)));

    //    Command m_autonomousCommand = (new AutonDriveCommand(100, -0.2).andThen(new AutonDriveCommand(100, 0.2))
    //    .andThen(new AutonStrafeCommand(-0.2, pipelineNum)).andThen(new AutonArmCommand(false, 2, false))
    //    .andThen(new MoveForwardCommand(pipelineNum)).andThen(new AutonDriveCommand(15, -0.1)).deadlineWith(null)
    //    .andThen(new AutonTimedStrafeCommand(-0.2, 1.5)).andThen(new AutonDriveCommand(65, -0.5)).andThen(new AutoBalanceCommand()));
       return m_autonomousCommand;
    }
}
