// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.robot.testing;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/** Add your docs here. */
public class Logger{

    File logFile;
    private BufferedWriter writer;
    private String csv;

    public Logger(String filename) throws IOException{
        LocalDateTime myDateObj = LocalDateTime.now();
        DateTimeFormatter myFormatObj = DateTimeFormatter.ofPattern("dd-MM:HH:mm");
        logFile = new File("/U/logs/"+myFormatObj.format(myDateObj)+"_"+filename);
        logFile.createNewFile();
        writer = new BufferedWriter(new FileWriter(logFile));
        csv = "";
    }

    public void addData(String[] data){
        for(String mm : data){
            csv += mm + ",";
        }
        csv = csv.substring(0,csv.length()-1);
        csv+="\n";
    }

    public void writeCSV() throws IOException{
        writer.write(csv);
    }

    public void write(String data) throws IOException{
        writer.write(data);
        writer.newLine();
    }

    public void close() throws IOException{
        writer.close();
    }

}
