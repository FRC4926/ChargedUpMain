// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Joystick {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;
  }

  public static class ArmSetpoints {
    // HIGH
    public static final double highShoulder = 15;
    public static final double highForearm = 120;
    public static final double highWrist = 140;

    // LOW
    public static final double lowShoulder = -20;
    public static final double lowForearm = 60;
    public static final double lowWrist = 107.10;

    // FLOOR CONE INTAKE
    public static final double floorShoulderCone = 20;
    public static final double floorForearmCone = 20;
    public static final double floorWristCone = 70;

    // FLOOR CUBE INTAKE
    public static final double floorShoulderCube = 25;
    public static final double floorForearmCube = 20;
    public static final double floorWristCube = 66;

    // SUBSTATION
    public static final double substationShoulder = -25;
    public static final double substationForearm = 85;
    public static final double substationWrist = 155;

    // RESET
    public static final double resetShoulder = 0;
    public static final double resetForearm = 0;
    public static final double resetWrist = 0;

    // SINGLE SUB
    public static final double singleSubShoulder = 17;
    public static final double singleSubForearm = -60;
    public static final double singleSubWrist = 160;

  }


  
  public static final class RobotConstants {
    public static final double CAMERA_PITCH_RADIANS = 0;
    public static final double LIMELIGHT_MOUNT_HEIGHT = 35;
    public static final double WHEEL_DIAMETER = 8;
    public static final double GEAR_RATIO = 8.4; /*
                                                  * gear ratio for competition chassis: 8.4* & practice
                                                  * chassis: 9/
                                                  */
  }

  public static final class FieldConstants {
    // measurements in inches
    public static final double TOP_NODE_GOAL_HEIGHT = 41.875;
    public static final double MIDDLE_NODE_GOAL_HEIGHT = 22.125;
    public static final double APRILTAG_LOWER_HEIGHT = 14.55;
    public static final double APRILTAG_UPPER_HEIGHT = 52;// 23.375;
    public static final double APRILTAG_HEIGHT = 25;// 23.375;
  }

  // follow CAN IDs in ascending order
  public static class CanIDs {
    // drive IDs
    public static final int frontLeft1ID = 1;
    public static final int backLeft1ID = 2;
    public static final int frontRight1ID = 3;
    public static final int backRight1ID = 4;

    public static final int frontLeft2ID = 5;
    public static final int backLeft2ID = 6;
    public static final int frontRight2ID = 7;
    public static final int backRight2ID = 8;

    // intake IDs
    public static final int leftIntakeID = 9;
    public static final int rightIntakeID = 10;

    // arm IDs
    public static final int shoulderID = 9;
    public static final int forearmID = 10;
    public static final int wristID = 11;
    public static final int intakeID = 12;
  }

}
