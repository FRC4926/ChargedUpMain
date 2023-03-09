// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax frontLeft1;
  public CANSparkMax backLeft1;
  public CANSparkMax frontRight1;
  public CANSparkMax backRight1;
  public CANSparkMax frontLeft2;
  public CANSparkMax backLeft2;
  public CANSparkMax frontRight2;
  public CANSparkMax backRight2;

  public MotorControllerGroup frontLeft;
  public MotorControllerGroup backLeft;
  public MotorControllerGroup frontRight;
  public MotorControllerGroup backRight;

  MecanumDrive mecanumDrive;
  // MecanumDrive mecanumDrive2;

  public AHRS gyro;
  Rotation2d rotation2d;
  public RelativeEncoder frontLeftEncoder;
  public RelativeEncoder frontRightEncoder;
  public RelativeEncoder backLeftEncoder;
  public RelativeEncoder backRightEncoder;
 
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft1 = new CANSparkMax(Constants.CAN_IDs.frontLeftLeadID, MotorType.kBrushless);
    backLeft1 = new CANSparkMax(Constants.CAN_IDs.backLeftLeadID, MotorType.kBrushless);
    frontRight1 = new CANSparkMax(Constants.CAN_IDs.frontRightLeadID, MotorType.kBrushless);
    backRight1 = new CANSparkMax(Constants.CAN_IDs.backRightLeadID, MotorType.kBrushless);
    frontLeft2 = new CANSparkMax(Constants.CAN_IDs.frontLeftFollowerID, MotorType.kBrushless);
    backLeft2 = new CANSparkMax(Constants.CAN_IDs.backLeftFollowerID, MotorType.kBrushless);
    frontRight2 = new CANSparkMax(Constants.CAN_IDs.frontRightFollowerID, MotorType.kBrushless);
    backRight2 = new CANSparkMax(Constants.CAN_IDs.backRightFollowerID, MotorType.kBrushless);

    frontLeftEncoder = frontLeft1.getEncoder();
    frontRightEncoder = frontRight1.getEncoder();
    backLeftEncoder = backLeft1.getEncoder();
    backRightEncoder = backRight1.getEncoder();
    frontRight1.setInverted(true);
    backRight1.setInverted(true);
    frontRight2.setInverted(true);
    backRight2.setInverted(true);

    frontLeft = new MotorControllerGroup(frontLeft1, frontLeft2);
    backLeft = new MotorControllerGroup(backLeft1, backLeft2);
    frontRight = new MotorControllerGroup(frontRight1, frontRight2);
    backRight = new MotorControllerGroup(backRight1, backRight2);


    mecanumDrive = new MecanumDrive(frontLeft, backLeft2, frontRight, backLeft);

    // mecanumDrive = new MecanumDrive(frontLeftLead, backLeftLead, frontRightLead, backRightLead);
    // mecanumDrive2 = new MecanumDrive(frontLeftFollower, backLeftFollower, frontRightFollower, backRightFollower);


    gyro = new AHRS(Port.kMXP);
  }
  
  public void setCurrentLimits(int currentLimit){
    frontRight1.setSmartCurrentLimit(currentLimit);
    frontLeft1.setSmartCurrentLimit(currentLimit);
    backRight1.setSmartCurrentLimit(currentLimit);
    backLeft1.setSmartCurrentLimit(currentLimit);

    frontRight2.setSmartCurrentLimit(currentLimit);
    frontLeft2.setSmartCurrentLimit(currentLimit);
    backRight2.setSmartCurrentLimit(currentLimit);
    backLeft2.setSmartCurrentLimit(currentLimit);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public double getGyroPitch(){
    return gyro.getPitch() + 7.86 + 1.53;
  }

  /**
   * Drive command factory method.
   *
   * @return a command
   */
  public CommandBase DriveMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An Drive method querying a boolean state of the subsystem (for Drive, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean DriveCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  public void setBrake(){
    frontLeft1.setIdleMode(IdleMode.kBrake);
    frontRight1.setIdleMode(IdleMode.kBrake);
    backLeft1.setIdleMode(IdleMode.kBrake);
    backRight1.setIdleMode(IdleMode.kBrake);
    frontLeft2.setIdleMode(IdleMode.kBrake);
    frontRight2.setIdleMode(IdleMode.kBrake);
    backLeft2.setIdleMode(IdleMode.kBrake);
    backRight2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    frontLeft1.setIdleMode(IdleMode.kCoast);
    frontRight1.setIdleMode(IdleMode.kCoast);
    backLeft1.setIdleMode(IdleMode.kCoast);
    backRight1.setIdleMode(IdleMode.kCoast);
    frontLeft2.setIdleMode(IdleMode.kCoast);
    frontRight2.setIdleMode(IdleMode.kCoast);
    backLeft2.setIdleMode(IdleMode.kCoast);
    backRight2.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders(){
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
  }

  public void resetGyro(){
    gyro.reset();
  }



  public void drive(double forward, double strafe, double rotation, boolean isFieldOriented){
    rotation2d = Rotation2d.fromDegrees(gyro.getAngle());

    if(!isFieldOriented){
      mecanumDrive.driveCartesian(forward, strafe, rotation);
    }
    else{
      mecanumDrive.driveCartesian(forward, strafe, rotation, rotation2d);
    }
  }


  public double getAverageEncoderDistance(){
    double positionConversionFactor = Math.PI * Constants.Drive.wheelDiameter / Constants.ROBOT_CONSTANTS.GEAR_RATIO;
    return ((frontLeftEncoder.getPosition() * positionConversionFactor) + (frontRightEncoder.getPosition()* positionConversionFactor)) / 2;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
