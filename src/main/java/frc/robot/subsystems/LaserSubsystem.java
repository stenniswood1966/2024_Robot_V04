// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaserSubsystem extends SubsystemBase {
  private final TimeOfFlight rangeSensor = new TimeOfFlight(9);
  /** Creates a new LaserSubsystem. */
  public LaserSubsystem() {
    rangeSensor.setRangingMode(RangingMode.Short, 40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (rangeSensor.getRange() <= 250) { //number I picked from the air :)
      Constants.k_NoteisReady = true;
    }else {
      Constants.k_NoteisReady = false;
    }
    //System.out.println("Note Ready to shoot: " + Constants.k_NoteisReady);
    //System.out.println("Distance: " + (int)rangeSensor.getRange() + "mm   Std Dev: " + (int)rangeSensor.getRangeSigma() + "mm   Status: " + rangeSensor.getStatus());
    SmartDashboard.putBoolean("Note ready to shoot: ", Constants.k_NoteisReady);
  }
}
