// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubSystem. */
    private SparkMax AlgaeIntakeMotor;
    private SparkMax AlgaeIntakeAngleMotor;
    private SparkRelativeEncoder AlgaeIntakeRelEncoder;
    private SparkAbsoluteEncoder AlgaeIntakeAbsEncoder;
    private boolean HasAlgaeGP;
    private double IntakeAngle;
    private PIDController IntakeAnglePID;
    private DigitalInput DI_HasAlgae;
  public AlgaeIntakeSubsystem() {
    AlgaeIntakeAngleMotor = new SparkMax(10, MotorType.kBrushless);
    AlgaeIntakeAbsEncoder = AlgaeIntakeAngleMotor.getAbsoluteEncoder();

    AlgaeIntakeMotor = new SparkMax(11, MotorType.kBrushless);
    IntakeAnglePID = new PIDController(0.01, 0, 0);
    DI_HasAlgae = new DigitalInput(0);
  }
    
  @Override
  public void periodic() {
   IntakeAnglePID.setTolerance(2);
   IntakeAngle = AlgaeIntakeAbsEncoder.getPosition();
   double PIDOutput = IntakeAnglePID.calculate(IntakeAngle);
   AlgaeIntakeAngleMotor.set(PIDOutput);
   HasAlgaeGP = DI_HasAlgae.get();
  }
  public void RunIntakeIn(double speed){
      AlgaeIntakeMotor.set(speed);
  }
  public void RunIntakeOut(double speed){
      AlgaeIntakeAngleMotor.set(-speed);
  }
  public void StopIntake(){
  AlgaeIntakeMotor.set(0);
  }
  public void SetIntakeAngle(double AngleDeg){
IntakeAnglePID.setSetpoint(AngleDeg);
  }
  public boolean HasAlgae(){
    return HasAlgaeGP;
  } 
  public double GetIntakeAngle(){
    
    return AlgaeIntakeAbsEncoder.getPosition();
  }
  public double GetIntakeSpeed(){
    return AlgaeIntakeMotor.get();
  }
  public boolean IntakeInTolerance(){
    return false;
  }
}
