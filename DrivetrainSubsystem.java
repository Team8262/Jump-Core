/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.jumprobotics.robot.subsystems.Mk2SwerveDrivetrain;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private static DrivetrainSubsystem instance;
  
  Mk2SwerveDrivetrain drivetrain;

  public DrivetrainSubsystem() {
    double trackwidth = 19.5;
    double wheelbase = 23.5;
    boolean gyroscopeInverted = true;
    int[][] modulePorts = {{2, 1, 1}, //Front left steer, drive, encoder
                           {7, 8, 2}, //Front rigt steer, drive, encoder
                           {4, 3, 0}, //Back left steer, drive, encoder
                           {6, 5, 3}};  //Back right steer, drive, encoder
    double[] offsets = {0,    //Front left
                        0,    //Front right
                        0,    //Back left
                        0};    //Back right

    
    drivetrain = new Mk2SwerveDrivetrain(trackwidth, wheelbase, offsets, modulePorts, gyroscopeInverted);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented){
    drivetrain.drive(translation, rotation, fieldOriented);
  }

  public DrivetrainSubsystem getInstance(){
    if(instance == null){
      instance = new DrivetrainSubsystem();
    }
      return instance;
    
  }

  public void resetGyroscope(){
    drivetrain.resetGyroscope();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivetrain.updateSensors();
    drivetrain.updateStates();
    drivetrain.writeToSmartDashboard();
    
  }
}
