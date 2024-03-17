// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class ClimbSubsystem extends SubsystemBase {
  public static TalonFX motor1 = new TalonFX(7, TunerConstants.kCANbusName);

  // class member variable
  final VelocityVoltage m_velocity = new VelocityVoltage(0);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFX configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 152.0;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    
    motor1.getConfigurator().apply(fx_cfg, 0.050);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motor1.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround) { //could be ReverseLimitValue depends on rotation when climbing
      motor1.set(0.0);
    }

    }
 
  public void climb() {
   motor1.set(1.0);
   }

  public void set(Double speed)  {
  var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
  motor1.setControl(motorRequest); // Requests the motor to move
  }

   public void stop() {
    motor1.set(0.0);
   }
}
