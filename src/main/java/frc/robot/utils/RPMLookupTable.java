// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;

/** Add your docs here. */
public class RPMLookupTable {
    double distance;
    public RPMLookupTable(double distance){
        this.distance = distance;
        if (distance < 10){
            rpmTable.put(10.0, 1000.0);
        }
        
    }
    
    HashMap<Double, Double> rpmTable = new HashMap<Double, Double>();
    

    public double getRPM(double distance){
        return rpmTable.get(distance);
    }
}


/*
P:0.001
I: 3e-7
D: 0.035
FF: 0.000089285714

650 max output: 0.11
1200 max output: 0.19
1500 max output: 0.2
1700 max output: 0.24
2000 max output: 0.32
3250 max output: 0.51
*/