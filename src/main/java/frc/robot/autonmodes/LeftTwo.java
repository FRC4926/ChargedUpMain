// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonLimelightCommand;

/** Add your docs here. */
public class LeftTwo {
    public static int pipelineNum;

    public LeftTwo() {

        pipelineNum = 0;
        Subsystems.driveSubsystem.resetEncoders();
    }
    public static Command getCommand(){
       Command m_autonomousCommand = (new AutonArmCommand(false, 2).andThen(new AutonDriveCommand(10, 0.2))
       .andThen(new AutonArmCommand(false, 2)).andThen(new AutonDriveCommand(200, -0.4))
       .deadlineWith(new AutonArmCommand(false, 3)).andThen(new AutonArmCommand(false, 4))
       .andThen(new AutonDriveCommand(180, 0.4)).andThen(new AutonArmCommand(false, 2))
       .andThen(new AutonLimelightCommand(true)).andThen(new AutonArmCommand(false, 2)));
       return m_autonomousCommand;
    }
}
