// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.autoncommands.AutoBalanceCommand;
import frc.robot.autoncommands.TimedStrafe;
import frc.robot.autoncommands.AutonArmCommand;
import frc.robot.autoncommands.AutonDriveCommand;
import frc.robot.autoncommands.AutonIntakeCommand;

/** Add your docs here. */
public class OneConeLeftTaxi {
    public OneConeLeftTaxi(){
        Subsystems.driveSubsystem.resetEncoders();
    }

    public static Command getCommand(){
        return new AutonArmCommand(false, 2).andThen(new AutonIntakeCommand(0.45, 1))
        .andThen(new AutonDriveCommand(5, 0.1))
        .andThen(new AutonArmCommand(false, 3).alongWith(new TimedStrafe(-0.3, 0.35).andThen(new AutonDriveCommand(132, 0.4))));    
    }
}

