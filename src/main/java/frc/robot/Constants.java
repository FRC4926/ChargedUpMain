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

    public static final double highConeAngleShoulder = 23;
    public static final double highConeAngleForearm = -290;
    public static final double lowConeAngleShoulder = 58;
    public static final double lowConeAngleForearm = -210;
    public static final double highCubeAngleShoulder = 45;
    public static final double highCubeAngleForearm = -300;
    public static final double lowCubeAngleShoulder = 80;
    public static final double lowCubeAngleForearm = -240;

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
  public static class CAN_IDs {
    // drive IDs
    public static final int frontLeftLeadID = 1;
    public static final int backLeftLeadID = 2;
    public static final int frontRightLeadID = 3;
    public static final int backRightLeadID = 4;

    public static final int frontLeftFollowerID = 5;
    public static final int backLeftFollowerID = 6;
    public static final int frontRightFollowerID = 7;
    public static final int backRightFollowerID = 8;

    // intake IDs
    public static final int leftIntakeID = 9;
    public static final int rightIntakeID = 10;

    // arm IDs
    public static final int shoulderID = 11;
    public static final int elbowID = 12;
    public static final int gripID = 13;
    public static final int wristID = 14;
  }

}
