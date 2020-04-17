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
import java.io.Serializable;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>, Comparable<InterpolatingDouble>, Serializable {
    private static final long serialVersionUID = -4236134181182630613L;

    public Double value;

    public InterpolatingDouble(Double value) {
        this.value = value;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return Double.compare(value, other.value);
    }

    public InterpolatingDouble interpolate(InterpolatingDouble other, double t) {
        double delta = other.value - value;
        return new InterpolatingDouble(value + delta * t);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double delta = upper.value - value;
        double deltaQuery = query.value - value;

        return deltaQuery / delta;
    }
}