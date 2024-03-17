// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class FeedSubsystem extends SubsystemBase {
  private static TalonFX motor1 = new TalonFX(10, TunerConstants.kCANbusName);
  /** Creates a new FeedSubsytem. */
  public FeedSubsystem() {
    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFx Configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor1.getConfigurator().apply(fx_cfg, 0.050);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Feed() {
    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFx configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor1.getConfigurator().apply(fx_cfg, 0.050);

    motor1.set(0.8);
  }
  
  public void Intake() {
    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFx Configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor1.getConfigurator().apply(fx_cfg, 0.050);

    motor1.set(0.1);
  }

  public void Stop() {
    motor1.set(0);
  }
}
