// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.DataHighway;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import dev.doglog.*;
import edu.wpi.first.math.geometry.Pose2d;
public class DataHighwaySubsystem extends SubsystemBase {
  /** Creates a new DataHighwaySubsystem. */
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;
  private boolean hasCoral = false;
  private double ReefDistance = 0;
  private int ReefSegment = 0;
  private boolean InRedZone = true;
  private boolean InReefYellowZone = false;
  private boolean InCoralSationZone = false;
  private boolean AtCoralStation = false;
  private SwerveSubsystem Drive_SS;
  private CoralElevatorSubsystem Coral_SS;
  public DataHighwaySubsystem(SwerveSubsystem DriveSS, CoralElevatorSubsystem CoralSS) {
    Drive_SS = DriveSS;
    Coral_SS = CoralSS;
    m_ledBuffer = new AddressableLEDBuffer(142);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
     double ReefRadius = Units.inchesToMeters(76.5)/2;
    Pose2d  CurrentPose = Drive_SS.getPose();
    getDriveData();
    getCoralData();
    setDriveData();
    setCoralData();
    double distancefromReefWall = ReefDistance- ReefRadius;
    DogLog.log ("DistanceToReefWall", distancefromReefWall);
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern CoralRainbow = LEDPattern.rainbow(255,128);
    if (hasCoral){
      AtCoralStation = false;
      white.applyTo(m_ledBuffer);
      if (distancefromReefWall < 2 & distancefromReefWall > 1.25){
        yellow.applyTo(m_ledBuffer);
        InReefYellowZone = true;
      }
        

    }
    else
      green.applyTo(m_ledBuffer);
      if (distancefromReefWall < 2 & distancefromReefWall > 1.25){
        yellow.applyTo(m_ledBuffer);
        InReefYellowZone = true;
      }

    //red zone - reduced speed - no elevator functions
    InRedZone = false;  
    if (distancefromReefWall < 1.25 & distancefromReefWall > 0.0)
    {
      
      InRedZone = true;
    }
    if (CurrentPose.getX() > 8 && CurrentPose.getX() < 9.5)
    {
      //in climber zone
      InRedZone = true;
    }
    
    // TODO: redzone Coral Stations
    // 3.3m from 0 to 3.3m (on a 45)
   double BlueRightX = CurrentPose.getX();
   double BlueRightY = CurrentPose.getY();
    double BlueRightr = Math.sqrt(Math.pow(BlueRightX,2)+Math.pow(BlueRightY,2));
    InCoralSationZone = false;
    if (BlueRightr <2.5){
      InRedZone = true;
      InCoralSationZone =true;
    }
    else
    {
      InCoralSationZone = false;
      AtCoralStation = false;
    }
    
    if (InRedZone)
      red.applyTo(m_ledBuffer);
      
    if (AtCoralStation && !hasCoral)
    {
      //Ask for Coral
      CoralRainbow.applyTo(m_ledBuffer);
    }
    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);

  }
  private void getDriveData()
  {
    ReefDistance = Drive_SS.DH_Out_ReefDistance;
    ReefSegment = Drive_SS.DH_Out_ReefSegment;   
    AtCoralStation = Drive_SS.DH_Out_AtCoralStation; 
  }
  private void setDriveData(){
    Drive_SS.DH_In_HasCoral = hasCoral;
    Drive_SS.DH_In_InRedZone = InRedZone;
    Drive_SS.DH_In_CoralYellow = InReefYellowZone;
    Drive_SS.DH_InStationZone = InCoralSationZone;

    
  }
  private void getCoralData(){
    hasCoral = Coral_SS.DH_Out_HasCoral;
  }
  private void setCoralData(){
    Coral_SS.DH_In_CoralZone = InCoralSationZone;
    Coral_SS.DH_In_DistanceFromReef = ReefDistance;
    Coral_SS.DH_In_RedZone = InRedZone;
    Coral_SS.DH_In_YellowZone = InReefYellowZone;
    
  }
    // This method will be called once per scheduler run
  }


