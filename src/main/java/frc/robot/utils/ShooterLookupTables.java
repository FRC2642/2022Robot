// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import java.util.HashMap;

/** Add your docs here. */
public class ShooterLookupTables {
    final HashMap<Double, Double> shooterLookupTable = new HashMap<Double, Double>();

    public ShooterLookupTables() {
        shooterLookupTable.put(11.0, 1100.0);
        shooterLookupTable.put(22.0, 1100.0);
        shooterLookupTable.put(33.0, 1100.0);
        shooterLookupTable.put(44.0, 1100.0);
        shooterLookupTable.put(55.0, 1100.0);

    }
    public double lookupTables(double distance){
        return shooterLookupTable.get(distance);
    }
}

