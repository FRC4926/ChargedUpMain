// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autonmodes.AutonTesting;
import frc.robot.autonmodes.OneCube;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LimelightStrafeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static class Subsystems{
    public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final static ArmSubsystem2 armSubsystem2 = new ArmSubsystem2();
    public final static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public final static ArmSubsystem armSubsystem = new ArmSubsystem();
    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
  }

  public static class Commands{
    public final static DriveCommand driveCommand = new DriveCommand();
  }

  SendableChooser<String> m_chooser = new SendableChooser<>();
  String path;

  // public static Joystick driver = new Joystick(Constants.Joystick.kJoystickPort);
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(Constants.Joystick.kXboxPort);
  public static Joystick operator2 = new Joystick(2);

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    

    m_chooser.setDefaultOption("AutonTesting", "AutonTesting");
    m_chooser.addOption("TwoCubePark", "TwoCubePark");
    Shuffleboard.getTab("Autonomous").add(m_chooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(Subsystems.driveSubsystem::exampleCondition)
    //     .onTrue(new DriveCommand(driveSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(driveSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    path = m_chooser.getSelected();

    if(path.equals("TwoCubePark")){
      return OneCube.getCommand();
    }
    else if(path.equals("AutonTesting")){
      return AutonTesting.getCommand();
    }
    return null;
  }
}