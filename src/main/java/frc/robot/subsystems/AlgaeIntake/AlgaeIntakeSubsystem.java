// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Constants.DIO.AlgaeIntake;
import frc.robot.Robot;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubSystem. */
    private SparkMax algaeIntakeMotor;
    private SparkMax pivotMotor;
    
    private SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private double pivotAngle;

    private PIDController pivotPID;
    ArmFeedforward pivotFF = new ArmFeedforward(PIDs.AlgaeIntake.ks, PIDs.AlgaeIntake.kg, PIDs.AlgaeIntake.kv);
    double pivotOutput;

    private DigitalInput algaePresence;
    private boolean algaePresent;

  public AlgaeIntakeSubsystem() {
    pivotMotor = new SparkMax(CanIDs.AlgaeIntake.Pivot, MotorType.kBrushless);
    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

    algaeIntakeMotor = new SparkMax(CanIDs.AlgaeIntake.BallIntake, MotorType.kBrushless);
    pivotPID = new PIDController(PIDs.AlgaeIntake.kp, PIDs.AlgaeIntake.ki, PIDs.AlgaeIntake.kd);
    algaePresence = new DigitalInput(AlgaeIntake.AlgaePresence);
  }
    
  @Override
  public void periodic() {
    ReadSensorValues();

    pivotOutput = pivotPID.calculate(pivotAngle) + Math.toDegrees(pivotFF.calculate (Math.toRadians(pivotAbsoluteEncoder.getPosition()), PIDs.AlgaeIntake.maxVelocity));
    // pivotMotor.set(pivotOutput);
  }
  public void RunIntakeIn(double speed){
      algaeIntakeMotor.set(speed);
  }
  public void RunIntakeOut(double speed){
    pivotMotor.set(-speed);
  }
  public void StopIntake(){
  algaeIntakeMotor.set(0);
  }
  public double GetIntakeSpeed(){
    return algaeIntakeMotor.get();
  }
  public boolean IntakeInTolerance(){
    return false;
  }
  public boolean HasAlgae() {
    return algaePresent;
  }

  private Boolean CanMoveIntakeInc(){
    return pivotAngle < RobotLimits.AlgaeIntake.maxAngle;
  }
  private Boolean CanMoveIntakeDec(){
    return pivotAngle > RobotLimits.AlgaeIntake.minAngle;
  }
  public void SetIntakeAngle(double angle)
  {
    if (angle > pivotPID.getSetpoint() && CanMoveIntakeInc()){
      pivotPID.setSetpoint(angle);
    }
    if (angle < pivotPID.getSetpoint() && CanMoveIntakeDec()){
      pivotPID.setSetpoint(angle);
    }
  }
  public double GetIntakeAngle() {
    return pivotAngle;
  }
  
   public SysIdRoutine PivotSysIDRun(Config config)
  {
    return new SysIdRoutine(
    config,
    new SysIdRoutine.Mechanism(
      (voltage) -> this.sysidPivotRunVoltage(voltage.in(Volts)),
      null, // No log consumer, since data is recorded by URCL
      this)
      );
  }
  public void sysidPivotRunVoltage(double V)
  {
    // if (CanMoveIntakeInc() && V > 0)
    //   pivotMotor.setVoltage(V);
    // else if (CanMoveIntakeDec() && V < 0)
    //   pivotMotor.setVoltage(V);
    // else
    //   pivotMotor.setVoltage(0);
  }

  private void ReadSensorValues() {
    algaePresent = algaePresence.get();
    pivotAngle = pivotAbsoluteEncoder.getPosition();
  }
}
