// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(0, "rio");
    private final int LedCount = 70;
    private boolean k_NoteisReadyPast = false;
 
    public LEDSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.75;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);

        startup();
    }

  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Only change LEDs if needed.
        if (Constants.k_NoteisReady && !k_NoteisReadyPast) {
            green();
            k_NoteisReadyPast = Constants.k_NoteisReady;
        } else if (!Constants.k_NoteisReady && k_NoteisReadyPast) {
            red();
            k_NoteisReadyPast = Constants.k_NoteisReady;
        } else {
            k_NoteisReadyPast = Constants.k_NoteisReady;
        }
    }


    public void red() {
        m_candle.setLEDs(255, 0, 0, 0, 0, LedCount);
    }

    public void green() {
        m_candle.setLEDs(0, 255, 0, 0, 0, LedCount);
    }

    public void startup() {
        m_candle.setLEDs(255, 255, 0, 0, 0, 70);
    }

}
