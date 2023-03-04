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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax frontLeftLead;
  public CANSparkMax backLeftLead;
  public CANSparkMax frontRightLead;
  public CANSparkMax backRightLead;
  public CANSparkMax frontLeftFollower;
  public CANSparkMax backLeftFollower;
  public CANSparkMax frontRightFollower;
  public CANSparkMax backRightFollower;

  MecanumDrive mecanumDrive;
  MecanumDrive mecanumDrive2;

  public AHRS gyro;
  Rotation2d rotation2d;
  public RelativeEncoder frontLeftEncoder;
  public RelativeEncoder frontRightEncoder;
  public RelativeEncoder backLeftEncoder;
  public RelativeEncoder backRightEncoder;
 
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeftLead = new CANSparkMax(Constants.CAN_IDs.frontLeftLeadID, MotorType.kBrushless);
    backLeftLead = new CANSparkMax(Constants.CAN_IDs.backLeftLeadID, MotorType.kBrushless);
    frontRightLead = new CANSparkMax(Constants.CAN_IDs.frontRightLeadID, MotorType.kBrushless);
    backRightLead = new CANSparkMax(Constants.CAN_IDs.backRightLeadID, MotorType.kBrushless);
    frontLeftFollower = new CANSparkMax(Constants.CAN_IDs.frontLeftFollowerID, MotorType.kBrushless);
    backLeftFollower = new CANSparkMax(Constants.CAN_IDs.backLeftFollowerID, MotorType.kBrushless);
    frontRightFollower = new CANSparkMax(Constants.CAN_IDs.frontRightFollowerID, MotorType.kBrushless);
    backRightFollower = new CANSparkMax(Constants.CAN_IDs.backRightFollowerID, MotorType.kBrushless);

    frontLeftEncoder = frontLeftLead.getEncoder();
    frontRightEncoder = frontRightLead.getEncoder();
    backLeftEncoder = backLeftLead.getEncoder();
    backRightEncoder = backRightLead.getEncoder();
    frontRightLead.setInverted(true);
    backRightLead.setInverted(true);
    frontRightFollower.setInverted(true);
    backRightFollower.setInverted(true);

    

    mecanumDrive = new MecanumDrive(frontLeftLead, backLeftLead, frontRightLead, backRightLead);
    mecanumDrive2 = new MecanumDrive(frontLeftFollower, backLeftFollower, frontRightFollower, backRightFollower);


    gyro = new AHRS(Port.kMXP);
  }
  
  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public double getGyroPitch(){
    return gyro.getPitch() + 7.86 + 1.53;
  }

  public double getGyroRoll(){
    return gyro.getRoll();
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
    frontLeftLead.setIdleMode(IdleMode.kBrake);
    frontRightLead.setIdleMode(IdleMode.kBrake);
    backLeftLead.setIdleMode(IdleMode.kBrake);
    backRightLead.setIdleMode(IdleMode.kBrake);
    frontLeftFollower.setIdleMode(IdleMode.kBrake);
    frontRightFollower.setIdleMode(IdleMode.kBrake);
    backLeftFollower.setIdleMode(IdleMode.kBrake);
    backRightFollower.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    frontLeftLead.setIdleMode(IdleMode.kCoast);
    frontRightLead.setIdleMode(IdleMode.kCoast);
    backLeftLead.setIdleMode(IdleMode.kCoast);
    backRightLead.setIdleMode(IdleMode.kCoast);
    frontLeftFollower.setIdleMode(IdleMode.kCoast);
    frontRightFollower.setIdleMode(IdleMode.kCoast);
    backLeftFollower.setIdleMode(IdleMode.kCoast);
    backRightFollower.setIdleMode(IdleMode.kCoast);
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
      mecanumDrive2.driveCartesian(forward, strafe, rotation);
    }
    else{
      mecanumDrive.driveCartesian(forward, strafe, rotation, rotation2d);
      mecanumDrive2.driveCartesian(forward, strafe, rotation, rotation2d);
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
