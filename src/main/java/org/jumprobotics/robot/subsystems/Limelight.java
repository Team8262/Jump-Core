// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
    NetworkTable table;
    NetworkTableEntry xOffset;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    double limelightX;
    double limelightY;
    double limelightArea;
    public double targetExists;
    double distance;

    // inches
    double heightDiff;
    double limelightHeight;
    double targetHeight;

    double limelightAngle;

    double defaultedDistance; //Distance that Limelight returns when no target is found
    

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        xOffset = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

    public Limelight SET_HEIGHTS(double limelight, double goal){
      this.limelightHeight = limelight;
      this.targetHeight = goal;
      this.heightDiff = goal = limelight;
      return this;
    }

    public Limelight SET_ANGLE(double angle){
      this.limelightAngle = angle;
      return this;
    }

    public Limelight SET_DEFAULT_DISTANCE(double dist){
      this.defaultedDistance = dist;
      return this;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        limelightX = xOffset.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        limelightArea = ta.getDouble(0.0);
        targetExists = ((tv.getDouble(0.0)==1) ? 1d : 0d);
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getDistance() {
        double newDistance = 1*this.heightDiff/Math.tan((limelightY+this.limelightAngle)*Math.PI/180);
        if(Math.abs(newDistance - this.defaultedDistance) < 0.1 ) {
            return distance;
        }
        distance = newDistance;
        return distance;
        //return 1*limelightHeightDifference/Math.tan((limelightY+Constants.LIMELIGHTANGLE)*Math.PI/180);
    }

    public double getXOffset() {
        return limelightX; // return the x-offset from the camera to reflective tape
    }
}
}
