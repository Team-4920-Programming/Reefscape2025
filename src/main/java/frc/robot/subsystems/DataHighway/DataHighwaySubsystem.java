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
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import dev.doglog.*;
public class DataHighwaySubsystem extends SubsystemBase {
  /** Creates a new DataHighwaySubsystem. */
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;
  private boolean hasCoral = false;
  private double ReefDistance = 0;
  private int ReefSegment = 0;
  private SwerveSubsystem Drive_SS;
  private CoralElevatorSubsystem Coral_SS;
  public DataHighwaySubsystem(SwerveSubsystem DriveSS, CoralElevatorSubsystem CoralSS) {
    Drive_SS = DriveSS;
    Coral_SS = CoralSS;
    m_ledBuffer = new AddressableLEDBuffer(20);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    getDriveData();
    getCoralData();
    setDriveData();
    setCoralData();
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    if (hasCoral){
      white.applyTo(m_ledBuffer);
      if (ReefDistance < 2 & ReefDistance > 1)
        yellow.applyTo(m_ledBuffer);
      if (ReefDistance < 1 & ReefDistance > 0.5)
        red.applyTo(m_ledBuffer);
    }
    else
      green.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);

  }
  private void getDriveData()
  {
    ReefDistance = Drive_SS.DH_Out_ReefDistance;
    ReefSegment = Drive_SS.DH_Out_ReefSegment;    
  }
  private void setDriveData(){
    Drive_SS.DH_In_HasCoral = hasCoral;
  }
  private void getCoralData(){
    hasCoral = Coral_SS.DH_Out_HasCoral;
  }
  private void setCoralData(){
    Coral_SS.DH_In_DistanceFromReef = ReefDistance;
    
  }
    // This method will be called once per scheduler run
  }


