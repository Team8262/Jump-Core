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
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.jumprobotics.robot.control.PidConstants;
import org.jumprobotics.robot.control.PidController;
import org.jumprobotics.robot.drivers.SwerveModule;
import org.jumprobotics.robot.math.Vector2;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


public class Mk2SwerveModuleFalcon extends SwerveModule {
    /**
     * The default drive encoder rotations per unit.
     */
    public static final double DEFAULT_DRIVE_ROTATIONS_PER_UNIT = (1.0 / (4.0 * Math.PI)) * (60.0 / 15.0) * (18.0 / 26.0) * (/*42.0*/2048.0 / 14.0);

    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    private static final double CAN_UPDATE_RATE = 50.0;

    private final double angleOffset;

    private TalonFX steeringMotor;
    private AnalogInput angleEncoder;
    private TalonFX driveMotor;
    //private CANEncoder driveEncoder;

    private final Object canLock = new Object();
    private double driveDistance = 0.0;
    private double drivePercentOutput = 0.0;
    private double driveVelocity = 0.0;
    private double driveCurrent = 0.0;

    private double driveEncoderRotationsPerUnit = DEFAULT_DRIVE_ROTATIONS_PER_UNIT;

    /**
     * All CAN operations are done in a separate thread to reduce latency on the control thread
     */
    private Notifier canUpdateNotifier = new Notifier(() -> {
        //double driveRotations = driveEncoder.getPosition();
        double driveRotations = driveMotor.getSensorCollection().getIntegratedSensorPosition();
        synchronized (canLock) {
            driveDistance = driveRotations * (1.0 / driveEncoderRotationsPerUnit);
        }

        //double driveRpm = driveEncoder.getVelocity();
        double driveRpm = driveMotor.getSensorCollection().getIntegratedSensorVelocity();
        synchronized (canLock) {
            driveVelocity = driveRpm * (1.0 / 60.0) * (1.0 / driveEncoderRotationsPerUnit);
        }

        double localDriveCurrent = driveMotor.getOutputCurrent();
        synchronized (canLock) {
            driveCurrent = localDriveCurrent;
        }

        double localDrivePercentOutput;
        synchronized (canLock) {
            localDrivePercentOutput = drivePercentOutput;
        }
        driveMotor.set(TalonFXControlMode.PercentOutput,localDrivePercentOutput);
    });

    private PidController angleController = new PidController(ANGLE_CONSTANTS);

    /**
     * @param modulePosition The module's offset from the center of the robot's center of rotation
     * @param angleOffset    An angle in radians that is used to offset the angle encoder
     * @param angleMotor     The motor that controls the module's angle
     * @param driveMotor     The motor that drives the module's wheel
     * @param angleEncoder   The analog input for the angle encoder
     */
    public Mk2SwerveModuleFalcon(Vector2 modulePosition, double angleOffset,
                           TalonFX angleMotor, TalonFX driveMotor, AnalogInput angleEncoder) {
        super(modulePosition);
        this.angleOffset = angleOffset;
        this.steeringMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.driveMotor = driveMotor;
        //this.driveEncoder = new CANEncoder(driveMotor);

        //driveMotor.setSmartCurrentLimit(60);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,60, 0, 0));

        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

        angleController.setInputRange(0.0, 2.0 * Math.PI);
        angleController.setContinuous(true);
        angleController.setOutputRange(-0.5, 0.5);

        canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
    }

    @Override
    protected double readAngle() {
        double angle = (1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + angleOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public void setFramePeriod(int periodMs){
        steeringMotor.setStatusFramePeriod(1, periodMs);/// 1 refers to Status_1_General
        driveMotor.setStatusFramePeriod(1, periodMs);

        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_status_frame.html#afc5d46cedacf46e01da84b3c0d3b9644
        ///Make sure that the frameValue is correct, I dunno if it's correct
        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_status_frame.html#afc5d46cedacf46e01da84b3c0d3b9644


        steeringMotor.setControlFramePeriod(3, periodMs);/// 3 refers to Control_3_General 
        driveMotor.setControlFramePeriod(3, periodMs);

        ///https://www.ctr-electronics.com/downloads/api/java/html/interfacecom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_i_motor_controller.html#af5da9318fb4e366f03f9b623c6d1c67d
        ///https://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_control_frame.html
        ///Make sure that the frameValue is correct, I dunno if it's correct
    }

    @Override
    protected double readDistance() {
        synchronized (canLock) {
            return driveDistance;
        }
    }

    protected double readVelocity() {
        synchronized (canLock) {
            return driveVelocity;
        }
    }

    protected double readDriveCurrent() {
        double localDriveCurrent;
        synchronized (canLock) {
            localDriveCurrent = driveCurrent;
        }

        return localDriveCurrent;
    }

    @Override
    public double getCurrentVelocity() {
        return readVelocity();
    }

    @Override
    public double getDriveCurrent() {
        return readDriveCurrent();
    }

    @Override
    protected void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
    }

    @Override
    protected void setDriveOutput(double output) {
        synchronized (canLock) {
            this.drivePercentOutput = output;
        }
    }

    @Override
    public void updateState(double dt) {
        super.updateState(dt);

        steeringMotor.set(TalonFXControlMode.PercentOutput,angleController.calculate(getCurrentAngle(), dt));
    }

    public void setDriveEncoderRotationsPerUnit(double driveEncoderRotationsPerUnit) {
        synchronized (canLock) {
            this.driveEncoderRotationsPerUnit = driveEncoderRotationsPerUnit;
        }
    }
}