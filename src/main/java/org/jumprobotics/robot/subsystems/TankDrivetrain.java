/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.subsystems;

/**
 * Add your docs here.
 */
import org.jumprobotics.robot.math.Vector2;

public abstract class TankDrivetrain extends Drivetrain {
    private final double trackWidth;

    public TankDrivetrain(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public final double getTrackWidth() {
        return trackWidth;
    }

    public abstract double getLeftDistance();
    public abstract double getRightDistance();

    public void arcadeDrive(double forward, double turn) {
        tankDrive(forward + turn, forward - turn);
    }

    public abstract void tankDrive(double left, double right);

    @Override
    public synchronized void updateKinematics(double timestamp) {
        // TODO: Tank drive kinematics
    }

    @Override
    public Vector2 getKinematicPosition() {
        return Vector2.ZERO; // TODO: Tank drive kinematics
    }

    @Override
    public Vector2 getKinematicVelocity() {
        return Vector2.ZERO; // TODO: Tank drive kinematics
    }

    @Override
    public void stop() {
        tankDrive(0, 0);
    }
}
