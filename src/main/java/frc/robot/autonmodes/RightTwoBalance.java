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
import frc.robot.autoncommands.AutonStrafeCommand;
import frc.robot.autoncommands.AutonTimedStrafeCommand;
import frc.robot.autoncommands.ForwardDistance;
import frc.robot.autoncommands.MoveForwardCommand;
import frc.robot.autoncommands.TimedStrafe;
import frc.robot.autoncommands.VisionCommand;


/** Add your docs here. */
public class RightTwoBalance {
    public static int pipelineNum;

    public RightTwoBalance() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
        

       Command m_autonomousCommand = (new AutonDriveCommand(100, -0.2).andThen(new AutonDriveCommand(100, 0.2))
       .andThen(new AutonStrafeCommand(0.2, pipelineNum)).andThen(new AutonArmCommand(false, 2, false))
       .andThen(new MoveForwardCommand(pipelineNum)) // <------ need to tune moveforward bc of new limelight placement.
       .andThen(new AutonDriveCommand(15, -0.1)).andThen(new AutonTimedStrafeCommand(0.2, 1.7))
       .andThen(new AutonDriveCommand(65, -0.5)).andThen(new AutoBalanceCommand()));
       return m_autonomousCommand;
    }
}
