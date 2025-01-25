// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax ClimberMotor;
  private SparkRelativeEncoder ClimberRelEncoder;
  private DigitalInput ClimberHomeSwitch;
  private boolean IsClimberHome;
  private double ClimberPosition;


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    ClimberMotor = new SparkMax(17, MotorType.kBrushless);
    ClimberHomeSwitch = new DigitalInput(2); 
    
  }

  @Override
  public void periodic() {
    IsClimberHome = ClimberHomeSwitch.get();
    ClimberPosition = ClimberRelEncoder.getPosition();
    if(IsClimberHome == true)
    ClimberRelEncoder.setPosition(ClimberPosition);
  }
  public void StopClimber(){
    ClimberMotor.set(0);
  }
  public void RunClimberIn(double speed){
    ClimberMotor.set(speed);
  }
  public void RunClimberOut(double speed){
    ClimberMotor.set(-speed);
  }
  public boolean ClimberIsHome(){
    return IsClimberHome;
  }
  public double GetClimberPosition(){
    return ClimberPosition;
  }



}
