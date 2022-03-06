// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Comparator;

import org.opencv.features2d.BRISK;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class Vector2dComparator implements Comparator<Vector2d> {

    public Vector2dComparator(Axis axis){
        this.axis = axis;
    }
    private Axis axis;
    @Override
    public int compare(Vector2d o1, Vector2d o2) {
        switch (axis){
            case X:
                if (o1.x == o2.x) return 0;
                else return o1.x > o2.x ? 1 : -1;
            case Y:
                if (o1.y == o2.y) return 0;
                else return o1.y > o2.y ? 1 : -1;
        }
        return 0;
    }
    public enum Axis{
        X,
        Y
    }
}