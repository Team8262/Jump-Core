/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jumprobotics.robot.drivers.Gyroscope;
import org.jumprobotics.robot.math.Vector2;

public abstract class Drivetrain extends SubsystemBase {

  public void writeToLog() {}

	public void resetKinematics(double timestamp) {}

	public abstract void stop();
  
  public abstract Gyroscope getGyroscope();
	
	public abstract double getMaximumVelocity();
	public abstract double getMaximumAcceleration();

	public abstract void updateKinematics(double timestamp);

	public abstract Vector2 getKinematicPosition();

	public abstract Vector2 getKinematicVelocity();

	public void outputToSmartDashboard() {
		SmartDashboard.putString("Drivetrain position", getKinematicPosition().toString());
		SmartDashboard.putNumber("Drivetrain X velocity", getKinematicVelocity().x);
		SmartDashboard.putNumber("Drivetrain Y velocity", getKinematicVelocity().y);

		SmartDashboard.putNumber("Drivetrain angle", getGyroscope().getAngle().toDegrees());
	}

	public void zeroSensors() {}
}