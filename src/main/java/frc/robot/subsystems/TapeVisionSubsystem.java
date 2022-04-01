// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.RetroTapePipeline;

public class TapeVisionSubsystem extends SubsystemBase {

  private UsbCamera camera;
  private VisionThread visionthread;


  private final List<Vector2d> points = new ArrayList<Vector2d>();
  private double centerX;


  public static final Object imgLock = new Object();
  private static TapeVisionSubsystem instance; 

  /** Creates a new TapeVisionSubsystem. */
  public TapeVisionSubsystem() {
    instance = this;
    camera = CameraServer.startAutomaticCapture(1);

    camera.setFPS(10);
    camera.setResolution(320, 240);
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);


    visionthread = new VisionThread(camera, new RetroTapePipeline(), pipeline -> {
      
    points.clear();
      if (!pipeline.filterContoursOutput().isEmpty()) {
        for (var contour : pipeline.filterContoursOutput()) {
            Rect rect = Imgproc.boundingRect(contour);
            synchronized (imgLock) {
               points.add(new Vector2d( rect.x +(0.5*rect.width), rect.y +(0.5*rect.height)));
               centerX =  rect.x +(0.5*rect.width);
            }
          }
      }
    });
    
  
    visionthread.start();
  }

  //vision functions
  public static double getCenterX(){
    return instance.centerX;
  }
  public static double getNormalizedCenterX(){
    return (getCenterX()-80)/80;
  }
  public static int numDetections(){
    return instance.points.size();
  }

  public static void resetCenterX(){
    instance.centerX = 0;
  }
  

  @Override
  public void periodic() {
    String nums = "";
    for(int i = 0; i < points.size(); i++) nums+="["+points.get(i).x+"] ";
    SmartDashboard.putString("center x's", nums);
    SmartDashboard.putNumber("TAPE center x", getCenterX());

   // This method will be called once per scheduler run
  }
}
