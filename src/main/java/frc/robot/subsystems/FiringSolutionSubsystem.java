// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class FiringSolutionSubsystem extends SubsystemBase {

  private NavigableMap<Double, Double> shooterSpeeds = new TreeMap<Double, Double>();
  private NavigableMap<Double, Double> wristAngle = new TreeMap<Double, Double>();

  /** Creates a new FiringSolutionSubsystem. */
  public FiringSolutionSubsystem() {
    setUpSpeedLookUpTable();
    setUpWristLookUpTable();
  }

  private void setUpSpeedLookUpTable() {
    shooterSpeeds.put(0.0, (double) 50);
    shooterSpeeds.put(30.0, (double) 50);
    shooterSpeeds.put(35.0, (double) 50);
    shooterSpeeds.put(40.0, (double) 50);
    shooterSpeeds.put(45.0, (double) 50);
    shooterSpeeds.put(50.0, (double) 50);
    shooterSpeeds.put(55.0, (double) 55);
    shooterSpeeds.put(60.0, (double) 55);
    shooterSpeeds.put(65.0, (double) 55);
    shooterSpeeds.put(70.0, (double) 55);
    shooterSpeeds.put(75.0, (double) 60);
    shooterSpeeds.put(80.0, (double) 60);
    shooterSpeeds.put(85.0, (double) 60);
    shooterSpeeds.put(90.0, (double) 65);
    shooterSpeeds.put(95.0, (double) 65);
    shooterSpeeds.put(100.0, (double) 65);
    shooterSpeeds.put(105.0, (double) 65);
    shooterSpeeds.put(110.0, (double) 65);
    shooterSpeeds.put(115.0, (double) 65);
    shooterSpeeds.put(999.0, (double) 65);
  }

    private void setUpWristLookUpTable() {
    wristAngle.put(0.0, (double) 0.348);
    wristAngle.put(30.0, (double) 0.348);
    wristAngle.put(35.0, (double) 0.348);
    wristAngle.put(40.0, (double) 0.350);
    wristAngle.put(45.0, (double) 0.359);
    wristAngle.put(50.0, (double) 0.367);
    wristAngle.put(55.0, (double) 0.369);
    wristAngle.put(60.0, (double) 0.373);
    wristAngle.put(65.0, (double) 0.380);
    wristAngle.put(70.0, (double) 0.382);
    wristAngle.put(75.0, (double) 0.391);
    wristAngle.put(80.0, (double) 0.395);
    wristAngle.put(85.0, (double) 0.400);
    wristAngle.put(90.0, (double) 0.400);
    wristAngle.put(95.0, (double) 0.407);
    wristAngle.put(100.0, (double) 0.409);
    wristAngle.put(105.00, (double) 0.411);
    wristAngle.put(110.00, (double) 0.412);
    wristAngle.put(115.00, (double) 0.416);
    wristAngle.put(999.0, (double) 0.410);
  }

  private double getDistance() {
    //figure out the distance to the target
    //math to calculate the distance goes here (d = (h2-h1) / tan(a1+a2)

    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 23.5; 

    // distance from the target to the floor
    double goalHeightInches = 57.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    //Constants.k_LLDistanceToAprilTag = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  @Override
  public void periodic() {
    Constants.k_LLDistanceToAprilTag = getDistance();

    if (Constants.k_LLDistanceToAprilTag >= 36 && Constants.k_LLDistanceToAprilTag <= 115) {

    double s_closeDistance = shooterSpeeds.floorKey(Constants.k_LLDistanceToAprilTag);
    double s_farDistance = shooterSpeeds.ceilingKey(Constants.k_LLDistanceToAprilTag);
    double s_closeShooter = shooterSpeeds.floorEntry(Constants.k_LLDistanceToAprilTag).getValue();
    double s_farShooter = shooterSpeeds.ceilingEntry(Constants.k_LLDistanceToAprilTag).getValue();
    Constants.k_FiringSolutionSpeed = ((s_farShooter - s_closeShooter) / (s_farDistance - s_closeDistance))* (Constants.k_LLDistanceToAprilTag - s_farDistance) + s_farShooter;

    double closeDistance = wristAngle.floorKey(Constants.k_LLDistanceToAprilTag);
    double farDistance = wristAngle.ceilingKey(Constants.k_LLDistanceToAprilTag);
    double closeShooter = wristAngle.floorEntry(Constants.k_LLDistanceToAprilTag).getValue();
    double farShooter = wristAngle.ceilingEntry(Constants.k_LLDistanceToAprilTag).getValue();
    Constants.k_FiringSolutionAngle = ((farShooter - closeShooter) / (farDistance - closeDistance))* (Constants.k_LLDistanceToAprilTag - farDistance) + farShooter + Constants.k_WristModifyPosition;
    }
    else {
      Constants.k_FiringSolutionSpeed = Constants.k_ShootDefaultSpeed;
      Constants.k_FiringSolutionAngle = Constants.k_WristShootPosition + Constants.k_WristModifyPosition;
    }

  SmartDashboard.putNumber("FSS calculated distance: ", Constants.k_LLDistanceToAprilTag);
  SmartDashboard.putNumber("FSS speed: ", Constants.k_FiringSolutionSpeed);
  SmartDashboard.putNumber("FSS angle: ", Constants.k_FiringSolutionAngle);

  }
}
