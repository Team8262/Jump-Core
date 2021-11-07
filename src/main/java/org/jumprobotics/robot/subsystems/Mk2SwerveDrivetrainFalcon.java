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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jumprobotics.robot.drivers.Gyroscope;
import org.jumprobotics.robot.drivers.SwerveModule;
import org.jumprobotics.robot.math.Vector2;
import org.jumprobotics.robot.drivers.Mk2SwerveModuleBuilder;
import org.jumprobotics.robot.drivers.NavX;
import org.jumprobotics.robot.drivers.ADIS16470;

public class Mk2SwerveDrivetrainFalcon extends SubsystemBase {
  
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

    public double ANGLE_REDUCTION;
    public double DRIVE_REDUCTION;
    public double WHEEL_DIAMETER;

    public SwerveModule frontLeftModule,
                         frontRightModule,
                         backLeftModule,
                         backRightModule;
                         
    private SwerveDriveKinematics kinematics;


    public Gyroscope navGyroscope;
    public ADIS16470 adGyroscope;
    public boolean useNavX;


    public Mk2SwerveDrivetrainFalcon(double trackwidth, double wheelbase, double[] angleOffsets, int[][] modulePorts, boolean invertedGyroscope, boolean navXAvailable) {
            this(trackwidth, wheelbase, angleOffsets, modulePorts, invertedGyroscope, navXAvailable, 18.0 / 1.0, 8.31 / 1.0, 4.0);
  }

  public Mk2SwerveDrivetrainFalcon(double trackwidth, double wheelbase, double[] angleOffsets, int[][] modulePorts, boolean invertedGyroscope, boolean navXAvailable, double angleReduction, double driveReduction, double wheelDiameter){
        //Maybe we don't need these, but might as well right?
        ANGLE_REDUCTION = angleReduction;
        DRIVE_REDUCTION = driveReduction;
        WHEEL_DIAMETER = wheelDiameter;

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
            .angleMotor(new TalonFX(DRIVETRAIN_FL_STEER))
            .driveMotor(new TalonFX(DRIVETRAIN_FL_DRIVE))
            .build();
    frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_FR_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new TalonFX(DRIVETRAIN_FR_STEER))
            .driveMotor(new TalonFX(DRIVETRAIN_FR_DRIVE))
            .build();
    backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_BL_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new TalonFX(DRIVETRAIN_BL_STEER))
            .driveMotor(new TalonFX(DRIVETRAIN_BL_DRIVE))
            .build();
    backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(DRIVETRAIN_BR_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new TalonFX(DRIVETRAIN_BR_STEER))
            .driveMotor(new TalonFX(DRIVETRAIN_BR_DRIVE))
            .build();

    kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    if(navXAvailable){
            useNavX = true;
            navGyroscope = new NavX(SPI.Port.kMXP);
            navGyroscope.calibrate();
            navGyroscope.setInverted(invertedGyroscope);
    }else{
            useNavX = false;
            adGyroscope = new ADIS16470();
            adGyroscope.calibrate();
            adGyroscope.setInverted(invertedGyroscope);
    }

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

        SmartDashboard.putNumber("Gyroscope Angle", useNavX ? navGyroscope.getAngle().toDegrees() : adGyroscope.getAngle().toDegrees());
  }

  public void updateStates(){
        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
    rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
    double gyroAngle = useNavX ? navGyroscope.getAngle().toDegrees() : adGyroscope.getAngle().toDegrees();
    ChassisSpeeds speeds;
    if (fieldOriented) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                Rotation2d.fromDegrees(gyroAngle));
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
        if(useNavX){
                navGyroscope.setAdjustmentAngle(navGyroscope.getUnadjustedAngle());
        }else{
                adGyroscope.setAdjustmentAngle(adGyroscope.getUnadjustedAngle());
        }
}

public void setFramePeriod(int periodMs){///both control frame period and status frame period is changed
        TalonFX FL_Steer = new TalonFX(DRIVETRAIN_FL_STEER);
        TalonFX FL_Drive = new TalonFX(DRIVETRAIN_FL_DRIVE);
        TalonFX FR_Steer = new TalonFX(DRIVETRAIN_FR_STEER);
        TalonFX FR_Drive = new TalonFX(DRIVETRAIN_FR_Drive);
        TalonFX BL_Steer = new TalonFX(DRIVETRAIN_BL_STEER);
        TalonFX BL_Drive = new TalonFX(DRIVETRAIN_BL_DRIVE);
        TalonFX BR_Steer = new TalonFX(DRIVETRAIN_BR_STEER);
        TalonFX BR_Drive = new TalonFX(DRIVETRAIN_BR_Drive);

        FL_Steer.setStatusFramePeriod(1, periodMs);/// 1 refers to Status_1_General
        FL_Drive.setStatusFramePeriod(1, periodMs);
        FR_Steer.setStatusFramePeriod(1, periodMs);
        FR_Drive.setStatusFramePeriod(1, periodMs);
        BL_Steer.setStatusFramePeriod(1, periodMs);
        BL_Drive.setStatusFramePeriod(1, periodMs);
        BR_Steer.setStatusFramePeriod(1, periodMs);
        BR_Drive.setStatusFramePeriod(1, periodMs);
        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_status_frame.html#afc5d46cedacf46e01da84b3c0d3b9644
        ///Make sure that the frameValue is correct, I dunno if it's correct
        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_status_frame.html#afc5d46cedacf46e01da84b3c0d3b9644

        FL_Steer.setControlFramePeriod(3, periodMs);/// 3 refers to Control_3_General 
        FL_Drive.setControlFramePeriod(3, periodMs);
        FR_Steer.setControlFramePeriod(3, periodMs);
        FR_Drive.setControlFramePeriod(3, periodMs);
        BL_Steer.setControlFramePeriod(3, periodMs);
        BL_Drive.setControlFramePeriod(3, periodMs);
        BR_Steer.setControlFramePeriod(3, periodMs);
        BR_Drive.setControlFramePeriod(3, periodMs);
        ///https://www.ctr-electronics.com/downloads/api/java/html/interfacecom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_i_motor_controller.html#af5da9318fb4e366f03f9b623c6d1c67d
        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_control_frame.html
        ///Make sure that the frameValue is correct, I dunno if it's correct


    }

}