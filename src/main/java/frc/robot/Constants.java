// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {

    public static Rotation2d k_steering_target = new Rotation2d(Math.toRadians(0)); //used by AutoAlignCommand to rotate to target
    public static final double k_MMRange = .005; //this is the range MM is considered finished

    public static boolean k_NoteisReady = false;
    public static final double ShootDelayTime = 0.5; //how long to wait before feeding note into shooter
    public static final double HomeDelayTime = 0.5; //how long to wait after shooting to home shoulder and wrist

    //Shoulder MM postions
    public static boolean k_ShoulderMMisMoving = false;
    public static final double k_ShoulderHomePosition = 1;
    public static final double k_ShoulderShootPosition = 11; //default shooting position
    public static final double k_ShoulderAmpPosition = 46.775;

    //Wrist MM Position
    public static boolean k_WristMMisMoving = false;
    public static double k_WristHomePosition = 0.265;
    public static double k_WristShootPosition = 0.348; //default shooting position
    public static double k_WristAmpPosition = 0.612;
    public static double k_WristModifyPosition = 0.0; //used to modify the wrist angle at all setpoints. buttons 17 & 18 will adjust realtime.

    //shootersubsystem motor speeds
    public static double k_shootmotor1speed = 0.0;
    public static double k_shootmotor2speed = 0.0;
    public static double k_ShootDefaultSpeed = 50;

    //FiringSolutionSubsystem
    public static double k_LLDistanceToAprilTag = 0.0;
    public static double k_FiringSolutionSpeed = k_ShootDefaultSpeed; //set the initial values
    public static double k_FiringSolutionAngle = k_WristShootPosition; //set the initial values


    
}
