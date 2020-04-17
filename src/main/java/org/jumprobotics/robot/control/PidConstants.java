/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.control;

/**
 * Add your docs here.
 */
public final class PidConstants {
	public final double p, i, d;

	public PidConstants(double p, double i, double d) {
		this.p = p;
		this.i = i;
		this.d = d;
	}
}
