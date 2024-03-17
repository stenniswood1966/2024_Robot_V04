// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShoulderSubsystem extends SubsystemBase {
  static TalonFX motor1 = new TalonFX(5, TunerConstants.kCANbusName);
  static TalonFX motor2 = new TalonFX(14, TunerConstants.kCANbusName);

  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    //configure the TalonFX motors
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration

    //configuration for motor2
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

    //set motor2 as a strict follower
    //motor2.setControl(new StrictFollower(motor1.getDeviceID()));
    //apply configuration to motor2
    //motor2.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor


    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

    // set slot 0 gains
    var slot0Configs = fx_cfg.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.2; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 10; // A position error of 2.5 rotations results in 12 V output WAS 10
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    // set Motion Magic settings
    var motionMagicConfigs = fx_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 150; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 150; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1100; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor

    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor2.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motor1.getVelocity().getValueAsDouble() == 0.0) {
      Constants.k_ShoulderMMisMoving = false;
    }else {
      Constants.k_ShoulderMMisMoving = true;
    }

    SmartDashboard.putBoolean("MM Status - Shoulder: ", Constants.k_ShoulderMMisMoving);
    SmartDashboard.putNumber("Shoulder position: ", motor1.getPosition().getValueAsDouble());

  }

  public void enablemotionmagic(double targetpos) {
    // periodic, run Motion Magic with slot 0 configs,
    motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));

    motor2.setControl(mmReq.withPosition(targetpos).withSlot(0));



  }

  public void set(Double speed){
    var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
    motor1.setControl(motorRequest); // Requests the motor to move

    motor2.setControl(motorRequest); // Requests the motor to move
  }

  public void stop() {
    motor1.set(0);

     motor2.set(0);
  }

}
