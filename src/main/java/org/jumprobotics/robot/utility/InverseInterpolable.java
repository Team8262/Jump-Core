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
public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}