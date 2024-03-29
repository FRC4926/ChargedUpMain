// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.Subsystems;
// import frc.robot.autoncommands.VisionCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LimelightStrafeCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setFPS(15);
    // camera.setResolution(160, 120);
    
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    RobotContainer.isAutomated = true;

    m_robotContainer = new RobotContainer();
  }

  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if(RobotContainer.driver.getAButton()){
      Subsystems.armSubsystem.resetEncoders();
      Subsystems.armSubsystem.resetSetpoints();
    }
    SmartDashboard.putNumber("shoulder motor encoder", Subsystems.armSubsystem.shoulderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shoulder angle", Subsystems.armSubsystem.getDegreesShoulder());
    SmartDashboard.putNumber("forearm angle", Subsystems.armSubsystem.getDegreesForearm());
    SmartDashboard.putNumber("wrist angle", Subsystems.armSubsystem.getDegreesWrist());

    SmartDashboard.putNumber("gyro angle", Subsystems.driveSubsystem.getGyroAngle());


  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Subsystems.armSubsystem.setBrakeArm();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Subsystems.driveSubsystem.enableDriveVoltage();
    Subsystems.driveSubsystem.setCurrentLimits(60);
    Subsystems.driveSubsystem.setBrake();
    Subsystems.driveSubsystem.resetGyro();
    Subsystems.armSubsystem.resetEncoders();
    Subsystems.armSubsystem.resetSetpoints();
    Subsystems.armSubsystem.setBrakeArm();


    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Subsystems.driveSubsystem.disableDriveVoltage();
    Subsystems.armSubsystem.setBrakeArm();
    Subsystems.driveSubsystem.setRampRate(0.16667);
    Subsystems.driveSubsystem.setBrake();
    Subsystems.driveSubsystem.setCurrentLimits(35);
    Subsystems.driveSubsystem.resetEncoders();
    //--------------------------------------
    // Subsystems.armSubsystem.resetEncoders();
    Subsystems.armSubsystem.resetSetpoints(); // REMOVE THESE THREE LINES BEFORE COMP MATCH
    Subsystems.driveSubsystem.resetGyro();
    //--------------------------------------

    CommandScheduler.getInstance().schedule(new DriveCommand());
    CommandScheduler.getInstance().schedule(new BalanceCommand());
    CommandScheduler.getInstance().schedule(new ArmCommand());
    CommandScheduler.getInstance().schedule(new LimelightStrafeCommand());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
