/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jumprobotics.robot.drivers.Gyroscope;
import org.jumprobotics.robot.drivers.SwerveModule;
import org.jumprobotics.robot.math.Vector2;
import org.jumprobotics.robot.drivers.Mk2SwerveModuleBuilder;
import org.jumprobotics.robot.drivers.NavX;

public class Mk2SwerveDrivetrain extends SubsystemBase {
  
    private static double TRACKWIDTH;
    private static double WHEELBASE;

    private static double FRONT_LEFT_ANGLE_OFFSET;
    private static double FRONT_RIGHT_ANGLE_OFFSET;
    private static double BACK_LEFT_ANGLE_OFFSET;
    private static double BACK_RIGHT_ANGLE_OFFSET;

    public int DRIVETRAIN_FL_STEER;
    public int DRIVETRAIN_FL_DRIVE;

    public int DRIVETRAIN_FR_STEER; 
    public int DRIVETRAIN_FR_DRIVE; 

    public int DRIVETRAIN_BL_STEER; 
    public int DRIVETRAIN_BL_DRIVE;

    public int DRIVETRAIN_BR_STEER;
    public int DRIVETRAIN_BR_DRIVE;

    public int DRIVETRAIN_FL_ENCODER;
    public int DRIVETRAIN_FR_ENCODER; 
    public int DRIVETRAIN_BL_ENCODER;
    public int DRIVETRAIN_BR_ENCODER;

    private SwerveModule frontLeftModule,
                         frontRightModule,
                         backLeftModule,
                         backRightModule;
    private SwerveDriveKinematics kinematics;


    public final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
    public Mk2SwerveDrivetrain(double trackwidth, double wheelbase, double[] angleOffsets, int[][] modulePorts, boolean invertedGyroscope) {
        
        TRACKWIDTH = trackwidth;
        WHEELBASE = wheelbase;
    
        FRONT_LEFT_ANGLE_OFFSET = angleOffsets[0];
        FRONT_RIGHT_ANGLE_OFFSET = angleOffsets[1];
        BACK_LEFT_ANGLE_OFFSET = angleOffsets[2];
        BACK_RIGHT_ANGLE_OFFSET = angleOffsets[3];
    
        DRIVETRAIN_FL_STEER = modulePorts[0][0];
        DRIVETRAIN_FL_DRIVE = modulePorts[0][1];
        DRIVETRAIN_FL_ENCODER = modulePorts[0][2];

    
        DRIVETRAIN_FR_STEER = modulePorts[1][0]; 
        DRIVETRAIN_FR_DRIVE = modulePorts[1][1]; 
        DRIVETRAIN_FR_ENCODER = modulePorts[1][2]; 

    
        DRIVETRAIN_BL_STEER = modulePorts[2][0]; 
        DRIVETRAIN_BL_DRIVE = modulePorts[2][1];
        DRIVETRAIN_BL_ENCODER = modulePorts[2][2];

    
        DRIVETRAIN_BR_STEER = modulePorts[3][0];
        DRIVETRAIN_BR_DRIVE = modulePorts[3][1];
        DRIVETRAIN_BR_ENCODER = modulePorts[3][2];

        frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_FL_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(DRIVETRAIN_FL_STEER, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(DRIVETRAIN_FL_DRIVE, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_FR_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(DRIVETRAIN_FR_STEER, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(DRIVETRAIN_FR_DRIVE, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_BL_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(DRIVETRAIN_BL_STEER, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(DRIVETRAIN_BL_DRIVE, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_BR_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(DRIVETRAIN_BR_STEER, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(DRIVETRAIN_BR_DRIVE, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    
        gyroscope.calibrate();
        gyroscope.setInverted(invertedGyroscope);

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
  }
  
  //3 methods below to be called in periodic

  public void updateSensors(){
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();
  }

  public void writeToSmartDashboard(){
        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());
  }

  public void updateStates(){
        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
    rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
    ChassisSpeeds speeds;
    if (fieldOriented) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
    } else {
        speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
    frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
    backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
    backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
}

public void resetGyroscope() {
    gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
}

}