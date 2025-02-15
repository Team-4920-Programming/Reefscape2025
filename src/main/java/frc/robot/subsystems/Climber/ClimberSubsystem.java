// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climberMotor;

  private DigitalInput cagePresenceSensor;
  private boolean cagePresent;

  private AbsoluteEncoder climberAngleEncoder;
  private double climberAngle;

  PIDController ClimberPID = new PIDController(PIDs.Climber.kp, PIDs.Climber.ki, PIDs.Climber.kd);
  double climberOutput;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SparkMax(CanIDs.Climber.Winch, MotorType.kBrushless);
    cagePresenceSensor = new DigitalInput(DIO.Climber.CagePresence);
    climberAngleEncoder = climberMotor.getAbsoluteEncoder();
    
  }

  @Override
  public void periodic() {
    ReadSensorValues();

    climberOutput = ClimberPID.calculate(climberAngle);
    // climberMotor.set(climberOutput);
  }

  public void StopClimber(){
    climberMotor.set(0);
  }
  public void RunClimberIn(double speed){
    climberMotor.set(speed);
  }
  public void RunClimberOut(double speed){
    climberMotor.set(-speed);
  }

  private Boolean CanMoveClimberInc(){
    return climberAngle < RobotLimits.Climber.maxAngle;
  }
  private Boolean CanMoveClimberDec(){
    return climberAngle > RobotLimits.Climber.minAngle;
  }
  public void SetClimberAngle(double angle)
  {
    if (angle > ClimberPID.getSetpoint() && CanMoveClimberInc()){
      ClimberPID.setSetpoint(angle);
    }
    if (angle < ClimberPID.getSetpoint() && CanMoveClimberDec()){
      ClimberPID.setSetpoint(angle);
    }
  }
  
  public  SysIdRoutine ClimberSysIDRun(Config config)
  {
    return new SysIdRoutine(
    config,
    new SysIdRoutine.Mechanism(
      (voltage) -> this.sysidClimberRunVoltage(voltage.in(Volts)),
      null, // No log consumer, since data is recorded by URCL
      this)
      );
  }
  public void sysidClimberRunVoltage(double V)
  {
    // if (CanMoveClimberInc() && V > 0)
    //   climberMotor.setVoltage(V);
    // else if (CanMoveClimberDec() && V < 0)
    //   climberMotor.setVoltage(V);
    // else
    //   climberMotor.setVoltage(0);
  }

  private void ReadSensorValues() {
    climberAngle = climberAngleEncoder.getPosition();
    cagePresent = cagePresenceSensor.get();
  }
}
