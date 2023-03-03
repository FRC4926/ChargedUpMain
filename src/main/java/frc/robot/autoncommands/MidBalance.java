// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;

/** Add your docs here. */
public class MidBalance {
    public MidBalance(){
        Subsystems.driveSubsystem.resetEncoders();
    }

    public static Command getCommand(){
        Command m_autonomousCommand = (new AutonDriveCommand(65, 0.5).andThen(new AutoBalanceCommand()));
        return m_autonomousCommand;
    }
}
