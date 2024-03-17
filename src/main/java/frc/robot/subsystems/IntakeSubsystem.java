// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static TalonFX motor1 = new TalonFX(13,TunerConstants.kCANbusName);
  private static TalonFX motor2 = new TalonFX(8,TunerConstants.kCANbusName);

  private CANSparkMax motor3;
  private CANSparkMax motor4;

  public IntakeSubsystem() {
    //setup centering motors
    motor3 = new CANSparkMax(51, MotorType.kBrushless);
    motor4 = new CANSparkMax(52, MotorType.kBrushless);
    motor3.restoreFactoryDefaults();
    motor4.restoreFactoryDefaults();
    motor3.setInverted(true);
    motor4.setInverted(false);
    motor3.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor4.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //setup intake motors
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor1.getConfigurator().apply(fx_cfg, 0.050);

    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor2.getConfigurator().apply(fx_cfg, 0.050);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Intake() {
    //intake motors
    motor1.set(0.4);
    motor2.set(0.4);

    //centering motors
    motor3.set(0.2);
    motor4.set(0.2);
  }

  public void Outake() {
    //intake motors
    motor1.set(-0.4);
    motor2.set(-0.4);

    //centering motors
    motor3.set(-0.1);
    motor4.set(-0.1);
  }

  public void Stop() {
    motor1.set(0.0);
    motor2.set(0.0);
    motor3.set(0.0);
    motor4.set(0.0);
    
  }

}
