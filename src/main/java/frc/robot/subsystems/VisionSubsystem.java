// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.BlurContour;

public class VisionSubsystem extends SubsystemBase {
  private UsbCamera camera;
  private VisionThread visionthread;
  private Rect rect;
  private boolean isSquare;


  private double centerX;
  private double centerY;


  public static final Object imgLock = new Object();
  private static VisionSubsystem instance;
  /** Creates a new TapeVisionSubsystem. */
  public VisionSubsystem() {
    instance = this;
    camera = CameraServer.startAutomaticCapture(0);

    camera.setFPS(10);
    camera.setResolution(320, 240);
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    
    SmartDashboard.putNumber("tape cam fps actual?", camera.getActualFPS()); 

        //pdh.clearStickyFaults();
    double[] hue = null;
    double[] sat = null;
    double[] lum = null;
    
    var alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Blue) {
      hue = Constants.HSL_HUE_BLUE;
      sat = Constants.HSL_SAT_BLUE;
      lum = Constants.HSL_LUM_BLUE;
    }
    else if (alliance == Alliance.Red){
      hue = Constants.HSL_HUE_RED;
      sat = Constants.HSL_SAT_RED;
      lum = Constants.HSL_LUM_RED;
    }

    visionthread = new VisionThread(camera, new BlurContour(hue,sat,lum), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
              rect = r;
              //centerX = 2*r.x + r.width - (320/2);
              //centerY = 2*r.y + r.height - (240/2);
              if (Math.abs(rect.width - rect.height) < Constants.MIN_NUM_PIXELS_RECT_SIMILARITY){
                isSquare = true;
                
              
              centerX = r.x +(0.5*r.width);
              centerY = r.y +(0.5*r.height);
              }
              else { 
                isSquare = false;
                
              }
          }
      }
    });
  
    visionthread.start();
  }

  //vision methods
  public static double getCenterX(){
    return instance.centerX;
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("issquare", isSquare);
    SmartDashboard.putNumber("center ball x", centerX);
    // This method will be called once per scheduler run
  }
}
