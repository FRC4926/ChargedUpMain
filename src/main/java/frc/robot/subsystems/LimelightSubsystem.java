package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GalacPIDController2;

public class LimelightSubsystem extends SubsystemBase {

int pipelineNum = 0;
public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
boolean tv;
double tid;
double tx;
double ty;
double angleToGoalRadiansVertical;
public GalacPIDController2 pidController;
private boolean limelightIsDisabled=false;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
  }

  public void updateLimelightValues(){
    tv = table.getEntry("tv").getDouble(0.0)==1 ? true:false;
    tx=table.getEntry("tx").getDouble(0.0);
    ty=table.getEntry("ty").getDouble(0.0);
    tid=table.getEntry("tid").getDouble(0.0);
  }
  public double getTX(){
    return tx;
  }

  public double getTY(){
    return ty;
  }

  public boolean getTV(){
    return tv;
  }

  public double getAprilTagXDistance(){
    return(getUpperAprilTagZDistance()*Math.tan(Math.toRadians(tx)));
  }
  public double getTopNodeXDistance(){
    return(getTopNodeZDistance()*Math.tan(Math.toRadians(tx)));
  }
  public double getMiddleNodeXDistance(){
    return(getMiddleNodeZDistance()*Math.tan(Math.toRadians(tx)));
  }
  public double getUpperAprilTagZDistance(){
    //ty angle is given in degress 
    return((Constants.ROBOT_CONSTANTS.LIMELIGHT_MOUNT_HEIGHT-Constants.GAME_ARENA_INFO.APRILTAG_HEIGHT))/Math.tan(Math.abs(Math.toRadians(ty-2)));
  }
  public double getTopNodeZDistance(){
    return (Constants.GAME_ARENA_INFO.TOP_NODE_GOAL_HEIGHT - Constants.ROBOT_CONSTANTS.LIMELIGHT_MOUNT_HEIGHT)/Math.tan(Math.toRadians(ty-2));
  }
  public double getMiddleNodeZDistance(){
    return (Constants.GAME_ARENA_INFO.MIDDLE_NODE_GOAL_HEIGHT - Constants.ROBOT_CONSTANTS.LIMELIGHT_MOUNT_HEIGHT)/Math.tan(Math.abs(Math.toRadians(ty-2)));
  }
  public void setPipeline(int pipeline) {
    pipelineNum = pipeline;
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);

  }
  public int getPipelineNum(){
    return pipelineNum;
  }
  @Override
  public void periodic() {
    if(!limelightIsDisabled){
      updateLimelightValues();
    }
 
    angleToGoalRadiansVertical = Math.toRadians(Constants.ROBOT_CONSTANTS.LIMELIGHT_MOUNT_HEIGHT + ty);
    // This method will be called once per scheduler run
  }
}