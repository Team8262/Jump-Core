/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.utility;

/**
 * Add your docs here.
 */
public class DrivetrainFeedforwardConstants {
    private final double velocityConstant;
    private final double accelerationConstant;
    private final double staticConstant;

    public DrivetrainFeedforwardConstants(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
    }

    public double calculateFeedforward(double velocity, double acceleration) {
        double feedforward = velocityConstant * velocity;
        feedforward += accelerationConstant * acceleration;

        feedforward += Math.copySign(staticConstant, feedforward);

        return feedforward;
    }

    public double getVelocityConstant() {
        return velocityConstant;
    }

    public double getAccelerationConstant() {
        return accelerationConstant;
    }

    public double getStaticConstant() {
        return staticConstant;
    }
}