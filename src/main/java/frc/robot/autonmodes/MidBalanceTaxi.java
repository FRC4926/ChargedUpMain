// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutoBalanceCommand;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonIntakeCommand;
import frc.robot.autoncommands.AutonTimedDrive;

/** Add your docs here. */
public class MidBalanceTaxi {
    public MidBalanceTaxi(){
        Subsystems.driveSubsystem.resetEncoders();
    }

    /**
     * @return
     */
    public static Command getCommand(){
        Command m_autonomousCommand = new AutonArmCommand(false, 2)
        .andThen(new AutonIntakeCommand(1.25, -0.5).deadlineWith(new AutonTimedDrive(0.5, -0.2))
        .andThen(new AutonDriveCommand(25, 0.35).deadlineWith(new AutonArmCommand(false, 3)))
        .andThen(new AutonDriveCommand(250, 0.4)).andThen(new AutonDriveCommand(100, -0.5)).andThen(new AutoBalanceCommand()));
        return m_autonomousCommand;
    }
}
