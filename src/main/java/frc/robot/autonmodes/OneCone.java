// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.MoveForwardCommand;

/** Add your docs here. */
public class OneCone {
    public static int pipelineNum;

    public OneCone() { 

        pipelineNum = 1;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
       Command m_autonomousCommand = (new AutonArmCommand(true, 2, false)
       .andThen(new AutonDriveCommand(10, 0.2)).andThen(new AutonArmCommand(true, 2, true)));
       return m_autonomousCommand;
    }
}
