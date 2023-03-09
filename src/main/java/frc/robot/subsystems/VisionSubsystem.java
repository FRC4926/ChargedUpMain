// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//@PreethamY
package frc.robot.subsystems;

//import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
//import java.util.Random;
//import javax.swing.JFrame;
//import javax.swing.JLabel;

import org.opencv.core.Core;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
//import org.opencv.objdetect.FaceDetectorYN;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

  // GalacPIDController2 PIDController;
  // double horizontalDistance;
  // public ConeAndBallNavigation(){
  //   PIDController = new GalacPIDController2(0.001, 0, 0, .01, () -> horizontalDistance, 0, 2);
  // }
  // public double getHorizontalDistance(){
  //   return horizontalDistance;
  // }
  // public void align(){
  //   Subsystems.driveSubsystem.mechDrive(0, 0, PIDController.getEffort(), true);
  // }
  Boolean see = false;
  double distance = 0;
  public double getDistance() {
    return distance;
  }
  int horizontalDistance = 0;
  public int getHorizontalDistance() {
    return horizontalDistance;
  }
  public boolean isSee(){
    return see;
  }
  static Thread m_visionThread;
  boolean cone = true;
//   public void senseCone(){
// cone = true;
//   }
//   public void senseCube(){
// cone = false;
//   }
  public void imageRunner() {
    m_visionThread =
    new Thread(
        () -> {

          Scalar upperY;
          Scalar lowerY;

          if(cone){
            upperY = new Scalar(30,255, 255);
            lowerY = new Scalar(15,120,120);
          }
          else{
            lowerY = new Scalar(90,50,70);
            upperY = new Scalar(132,255,255);
          }

          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
         
          // Set the resolution
          camera.setResolution(640, 480);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Yellow Object Detection", 640, 480);
         
          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();
          Mat keegan = new Mat();
          Mat blur = new Mat();
          Mat dst = new Mat();
          Mat hier = new Mat();
          // Mat grayMat = new Mat();
         
        List<MatOfPoint> points = new ArrayList<>();
          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            Imgproc.cvtColor(mat, keegan, Imgproc.COLOR_BGR2HSV);
            Imgproc.GaussianBlur(keegan, blur, new Size(11, 11), 0);
           
            Core.inRange(blur, lowerY, upperY, dst);
            // Imgproc.HoughCircles(blur, dst, 0, 0, 0);
           
            int cubeIndex = -1;

            Imgproc.findContours(dst, points, hier, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if(!cone){
            for(int i =0; i< points.size(); i++){  
                if((((double)Imgproc.boundingRect(points.get(i)).height/(double)Imgproc.boundingRect(points.get(i)).width)< 1.2 && ((double)Imgproc.boundingRect(points.get(i)).height/(double)Imgproc.boundingRect(points.get(i)).width )> 0.8) && Imgproc.boundingRect(points.get(i)).height>50 && Imgproc.boundingRect(points.get(i)).width>50){
                  cubeIndex = i;
                }  
             
             }
            }
         
            double maxVal = 0;
            int maxValIdx=-1;
           
            for(int countourIdx = 0; countourIdx<points.size(); countourIdx++){
              double contourArea = Imgproc.contourArea(points.get(countourIdx));
              if(maxVal<contourArea){
                maxVal = contourArea;
                maxValIdx = countourIdx;
              }
            }
         

          int centre = 0;
          if(points.size()>0){
            see = true;
            if(cone)
            centre = (Imgproc.boundingRect(points.get(maxValIdx)).x+(Imgproc.boundingRect(points.get(maxValIdx)).width/2) -320);
            else
            centre = (Imgproc.boundingRect(points.get(cubeIndex)).x+(Imgproc.boundingRect(points.get(cubeIndex)).width/2) -320);
            // horizontalDistance = centre/10;
          } else{
            see=false;
          }
            horizontalDistance = centre/5;
 
          // if(cone)
            Imgproc.drawContours(mat, points, cubeIndex, new Scalar(0,255,0), 4);
        //  else{
        //   Imgproc.drawContours(mat, points, cubeIndex, new Scalar(0,255,0), 4);
        //  }
           

         
           
            outputStream.putFrame(mat);
            points.clear();
            blur.empty();
            }
        });
m_visionThread.setDaemon(true);
m_visionThread.start();
  }
  public void changePipeline(){
cone = !cone;
  }

  public double getFocalLength(double physicalDistance, double realWidth, int pixelWidth)
  {
    return pixelWidth*physicalDistance/realWidth;
  }

  public double getDistance(double focalLength, double realWidth, int pixelWidth)
  {
    return realWidth*focalLength/pixelWidth;
  }
  @Override
  public void periodic() {
   
  }

}

