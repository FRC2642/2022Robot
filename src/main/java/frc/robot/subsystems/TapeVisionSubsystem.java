// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Vector2dComparator;

public class TapeVisionSubsystem extends SubsystemBase {

  private final ArrayList<Vector2d> tapeContourPositions = new ArrayList<Vector2d>();
  public static final int NUM_TAPE_ON_HUB = 6;
  public static final double TAPE_LENGTH_TOLERANCE = 0.25;
  public static final double TAPE_HEIGHT_TOLERANCE = 0.25;
  public static final double TAPE_NOISE_TOLERANCE = 0.25;
  public static final double TAPE_IN_LINE_TOLERANCE = 0.25;
  /** Creates a new VisionSubsystem. */
  //all vision (tape and ball) go in here (change variable names as necessary)
  public TapeVisionSubsystem() {}

  public ArrayList<Vector2d> getDetections(){
    return tapeContourPositions;
  }
  public void clearDetections(){
    tapeContourPositions.clear();
  }
  public void addDetection(Vector2d contourPosition){
    tapeContourPositions.add(contourPosition);
  }

  //vision processing
  public HubInFrameReason hubInFrame(Vector2d hubPosition, Vector2d hubSizeInFrame) throws Exception {

    if (hubPosition == null || hubSizeInFrame == null) throw new Exception("you failed.");
    if (tapeContourPositions.size() < NUM_TAPE_ON_HUB) return HubInFrameReason.NOT_ENOUGH_TAPES;

    //FIND HUB FROM POINTS
    //convert points to array
    Vector2d[] contours = new Vector2d[tapeContourPositions.size()];
    double[] cache = new double[tapeContourPositions.size()];
    for(int i = 0; i < tapeContourPositions.size(); ++i){
      contours[i] = tapeContourPositions.get(i);
    }
    //sort by nearest neighbor
    for (int i = 1; i < contours.length; ++i) {
      Object[] s = getClosestInSetFrom(contours, i-1); //get closest to the last point
      Vector2d original = contours[i]; //remember this point
      contours[i] = contours[(int)s[0]]; //swap
      contours[(int)s[0]] = original; //swap
      cache[i] = (double)s[1]; //remember the distance between this point and the last point
    }
    //take derivative of distances
    for (int i = 1; i < cache.length; ++i){
      cache[i - 1] = cache[i] - cache[i - 1];
    }
    cache[cache.length - 1] = Double.MAX_VALUE;

    //find groups
    ArrayList<Vector2d[]> sets = new ArrayList<Vector2d[]>();
    ArrayList<Vector2d> current = new ArrayList<>();
    
    for (int i = 0; i < cache.length; ++i) {
      if (cache[i] > TAPE_NOISE_TOLERANCE) { //if the change in distances between these two points is too large
        if (!current.isEmpty()) { //if weve been adding stuff
        current.add(contours[i]); //add the current item
        Vector2d[] vecArray = new Vector2d[current.size()]; // then cache it and clear the list
        sets.add(current.toArray(vecArray));
        current.clear();
        }
      }
      else{
        current.add(contours[i]);
      }
    }
    current.clear();

    //determine which groups could be the hub
    //are there enough tapes?
    ArrayList<Vector2d[]> largeEnoughSets = new ArrayList<Vector2d[]>();
    for (Vector2d[] set : sets) if (set.length > NUM_TAPE_ON_HUB) largeEnoughSets.add(set);
    if (largeEnoughSets.isEmpty()) return HubInFrameReason.NOT_ENOUGH_TAPES;

    //does it look long?
    ArrayList<Vector2d[]> longEnoughSets = new ArrayList<Vector2d[]>();
    for (Vector2d[] set : largeEnoughSets) {
      Arrays.sort(set, new Vector2dComparator(Vector2dComparator.Axis.X));
      if (Math.abs(set[set.length - 1].x - set[0].x) < TAPE_LENGTH_TOLERANCE) continue;
      Arrays.sort(set, new Vector2dComparator(Vector2dComparator.Axis.Y));
      if (Math.abs(set[set.length - 1].y - set[0].y)  > TAPE_HEIGHT_TOLERANCE) longEnoughSets.add(set);
    }
    if (longEnoughSets.isEmpty()) return HubInFrameReason.NO_SETS_MATCH_SIZE;

    //do any sets look like a line?
    for (Vector2d[] set : longEnoughSets) {
      //calculate average y of points
      double num = 0;
      for(Vector2d point : set) {
        num += point.y;
      }
      double avg = num / set.length;

      //see if any points stray too far away from the centerline
      boolean strayed = false;
      for(Vector2d point : set) {
        if (point.y - avg > TAPE_IN_LINE_TOLERANCE) {
          strayed = true;
          break;
        }
      }
      if (!strayed){ //if we didnt stray, this set passed all the tests

        Arrays.sort(set, new Vector2dComparator(Vector2dComparator.Axis.X));
        hubPosition.x = (set[set.length - 1].x + set[0].x)/2; //use midpoint formula for x 
        hubSizeInFrame.x = (set[0].x - set[set.length - 1].x);
        
        Arrays.sort(set, new Vector2dComparator(Vector2dComparator.Axis.Y));
        hubPosition.y = (set[set.length - 1].y + set[0].y)/2; //use midpoint formula for y 
        hubSizeInFrame.y = (set[0].y - set[set.length - 1].y);

        return HubInFrameReason.DETECTED;
      }
    }

    return HubInFrameReason.NOT_DETECTED;
  }
  private Object[] getClosestInSetFrom(Vector2d[] contours, int index){
      double smallestD = Double.MAX_VALUE;
      int smallestI = 0;
      for (int i = index + 1; i < contours.length; ++i){
        double d = Math.sqrt(contours[i].x*contours[i].x + contours[index].y*contours[index].y);
        if (d < smallestD) {
          smallestD = d;
          smallestI = i;
        }
      }
      return new Object[] { smallestI, smallestD };
  }
  
  enum HubInFrameReason{
    DETECTED,
    NOT_DETECTED,
    NOT_ENOUGH_TAPES,
    NO_SETS_MATCH_SIZE
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

