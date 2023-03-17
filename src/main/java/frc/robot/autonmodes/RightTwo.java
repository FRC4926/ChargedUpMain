// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import java.sql.Time;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutoBalanceCommand;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonLimelightCommand;
import frc.robot.autoncommands.TimedStrafe;
// import frc.robot.autoncommands.VisionCommand;


/** Add your docs here. */
public class RightTwo {
    public static int pipelineNum;

    public RightTwo() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
        

       Command m_autonomousCommand = (new AutonArmCommand(false, pipelineNum).andThen(new AutonDriveCommand(100, -0.2)).andThen(new AutonDriveCommand(100, 0.2))
       .andThen(new TimedStrafe(0.2, pipelineNum)).andThen(new AutonArmCommand(false, 2))
       .andThen(new AutonLimelightCommand(false)) // <------ need to tune moveforward bc of new limelight placement.
       .andThen(new AutonDriveCommand(15, -0.1)));
       return m_autonomousCommand;
    }
}
