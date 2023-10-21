// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonIntakeCommand;
import frc.robot.autoncommands.TimedStrafe;

/** Add your docs here. */
public class OneCubeTaxi {
    //public static int pipelineNum;

    public OneCubeTaxi() {

        //pipelineNum = 1;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
       Command m_autonomousCommand = new AutonArmCommand(false, 2)
       .andThen(new AutonIntakeCommand(0.35, -0.2)).andThen(new AutonArmCommand(false, 3))
       .andThen(new AutonDriveCommand(150, 0.4));
       return m_autonomousCommand;
    }
}
