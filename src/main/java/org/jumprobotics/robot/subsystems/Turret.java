// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  //Ratio of motor to turret mechanism (needed for translating motor encoder to turret angle)
  //Rotations of motor shaft per 1 rotation of turret
  private double TURRET_RATIO; 
  
  private CANSparkMax motor;
  private SparkMaxPIDController turncontroller;


  /** Creates a new Turret. */
  public Turret() {}

  public Turret SET_MOTOR(int can_id){
    this.motor = new CANSparkMax(can_id, MotorType.kBrushless);
    this.turncontroller = this.motor.getPIDController();
    return this;
  }

  public void setAngle(double angle){
    this.turncontroller.setReference((angle/360)*this.TURRET_RATIO, ControlType.kPosition);
  }

  public void setRawPosition(double pos){
    this.turncontroller.setReference(pos, ControlType.kPosition);
  }

  public Turret SET_LIMITS(double forwardLimit, double reverseLimit){
    if(this.TURRET_RATIO == 0){
      throw new NullPointerException("Shooter Ratio Not Set");
    }
    this.motor.setSoftLimit(SoftLimitDirection.kForward, (float) ((forwardLimit/360)*TURRET_RATIO));
    this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ((reverseLimit/360)*TURRET_RATIO));
    this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    return this;
  }

  public Turret SET_PID(double P, double I, double D){
    this.turncontroller.setP(P);
    this.turncontroller.setI(I);
    this.turncontroller.setD(D);
    return this;
  }

  public Turret SET_RAW_LIMITS(double forwardLimit, double reverseLimit){
    this.motor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardLimit);
    this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseLimit);
    this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    return this;
  }

  public CANSparkMax getMotor(){
    return this.motor;
  }

  public SparkMaxPIDController getController(){
    return this.turncontroller;
  }

  public Turret SET_RATIO(double ratio){
    this.TURRET_RATIO = ratio;
    return this;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
