// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShootSubsystem extends SubsystemBase {
  public static TalonFX motor1 = new TalonFX(11, TunerConstants.kCANbusName);
  public static TalonFX motor2 = new TalonFX(12, TunerConstants.kCANbusName);

  // class member variable
  final VelocityVoltage m_velocity = new VelocityVoltage(0);

  /** Creates a new ShootSubsystem. */
  public ShootSubsystem() {
    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFX configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // robot init, set slot 0 gains
    var slot0Configs = fx_cfg.Slot0;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.01;
    
    motor1.getConfigurator().apply(fx_cfg,0.050);

    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor2.getConfigurator().apply(fx_cfg, 0.050);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Constants.k_shootmotor1speed = motor1.getVelocity().getValueAsDouble();
    Constants.k_shootmotor2speed = motor2.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Shooter motor1 velocity: ", Constants.k_shootmotor1speed);
    SmartDashboard.putNumber("Shooter motor2 velocity: ", Constants.k_shootmotor2speed);
  }

  public void Shoot(double speed) {
    m_velocity.Slot = 0;
    motor1.setControl(m_velocity.withVelocity(speed));
    motor2.setControl(m_velocity.withVelocity(speed));
  }

  public void PreShoot() {
    m_velocity.Slot = 0;
    motor1.setControl(m_velocity.withVelocity(25));
    motor2.setControl(m_velocity.withVelocity(25));
  }

  public void AmpShoot() {
    m_velocity.Slot = 0;
    motor1.setControl(m_velocity.withVelocity(1));
    motor2.setControl(m_velocity.withVelocity(1));
  }

  public void Stop() {
    m_velocity.Slot = 0;
    motor1.setControl(m_velocity.withVelocity(0));
    motor2.setControl(m_velocity.withVelocity(0));
  }
  
}
