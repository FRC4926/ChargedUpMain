// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.AutonStrafeCommand;
import frc.robot.commands.AutonTimedStrafeCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.MoveForwardCommand;

/** Add your docs here. */
public class OneElementPark {
    public static int pipelineNum;

    public OneElementPark() {

        pipelineNum = 1;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){

        // Command m_autonomousCommand = (new AutonTimedStrafeCommand(0.2,1));
       Command m_autonomousCommand = (new AutonDriveCommand(100, 0.3).andThen(new AutonDriveCommand(100, -0.3))
       .andThen(new AutonStrafeCommand(0.2, pipelineNum)).andThen(new MoveForwardCommand(pipelineNum))
       .andThen(new AutonTimedStrafeCommand(0.2, 1.5)));
       return m_autonomousCommand;
    }
   

}
