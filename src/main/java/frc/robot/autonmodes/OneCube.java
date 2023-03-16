// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutoBalanceCommand;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonStrafeCommand;
import frc.robot.autoncommands.MoveForwardCommand;
import frc.robot.autoncommands.TimedStrafe;

/** Add your docs here. */
public class OneCube {
    public static int pipelineNum;

    public OneCube() {

        pipelineNum = 1;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){

       Command m_autonomousCommand = (new MoveForwardCommand(0).andThen(new AutonArmCommand(false, 2, true)));
       return m_autonomousCommand;
    }
   

}
