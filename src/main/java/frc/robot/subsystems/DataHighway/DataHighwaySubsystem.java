// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.DataHighway;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.*;
public class DataHighwaySubsystem extends SubsystemBase {
  /** Creates a new DataHighwaySubsystem. */
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;

  public DataHighwaySubsystem() {
    
    m_ledBuffer = new AddressableLEDBuffer(20);
    m_led.setLength(m_ledBuffer.getLength());



    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
   
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);

  }
    // This method will be called once per scheduler run
  }


