/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.drivers;

/**
 * Add your docs here.
 */
import org.jumprobotics.robot.drivers.Gyroscope;
import org.jumprobotics.robot.math.Rotation2;
import com.analog.adis16470.frc.ADIS16470_IMU;


public final class ADIS16470 extends Gyroscope {

    private final ADIS16470_IMU gyroscope;

    public ADIS16470(){
        gyroscope = new ADIS16470_IMU();
    }

    @Override
    public void calibrate() {
        gyroscope.reset();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        //return Rotation2.fromRadians(getAxis(Axis.YAW));
        return Rotation2.fromDegrees(gyroscope.getAngle());
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(gyroscope.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(gyroscope.getGyroInstantY());
            case ROLL:
                return Math.toRadians(gyroscope.getGyroInstantX());
            case YAW:
                return Math.toRadians(gyroscope.getGyroInstantZ());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}