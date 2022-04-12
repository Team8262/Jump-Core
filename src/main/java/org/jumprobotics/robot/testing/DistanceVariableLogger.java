// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.robot.testing;

import java.io.IOException;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class DistanceVariableLogger extends Logger{

    DoubleSupplier distance;
    DoubleSupplier variable;
    
    public DistanceVariableLogger(String filename, DoubleSupplier dist, DoubleSupplier variable) throws IOException{
        super(filename);
        this.distance = dist;
        this.variable = variable;
    }

    public void log(){
        addData(new String[]{Double.toString(distance.getAsDouble()), Double.toString(variable.getAsDouble())});
    }

    public void save() throws IOException{
        writeCSV();
    }


}


