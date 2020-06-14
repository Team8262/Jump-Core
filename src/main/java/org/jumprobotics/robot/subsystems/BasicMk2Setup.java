/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import org.jumprobotics.robot.drivers.SwerveModule;
import org.jumprobotics.robot.math.Vector2;
import org.jumprobotics.robot.drivers.Mk2SwerveModuleBuilder;
import java.util.HashMap;
/**
 * Add your docs here.
 */
public class BasicMk2Setup {
    public SwerveModule createModule(HashMap<String, Double> mappings){

        SwerveModule module = new Mk2SwerveModuleBuilder(
            new Vector2(mappings.get("trackwidth").doubleValue() / 2.0, mappings.get("wheelbase").doubleValue() / 2.0))
            .angleEncoder(new AnalogInput(mappings.get("encoder").intValue()), mappings.get("offset"))
            .angleMotor(new CANSparkMax(mappings.get("steer").intValue(), CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(mappings.get("drive").intValue(), CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

        return module;
    }
}
