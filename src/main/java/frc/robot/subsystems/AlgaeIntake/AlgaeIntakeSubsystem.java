// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Constants.DIO.AlgaeIntake;
import frc.robot.Robot;

import dev.doglog.*;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubSystem. */
  private SparkMax algaeIntakeMotor;
  private SparkMax pivotMotor;
  
  private SparkAbsoluteEncoder pivotAbsoluteEncoder;
  private double pivotAngle;

  private PIDController pivotPID;
  // ArmFeedforward pivotFF = new ArmFeedforward(PIDs.AlgaeIntake.ks, PIDs.AlgaeIntake.kg, PIDs.AlgaeIntake.kv);
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

    SmartDashboard.putNumber("pivot Output", pivotOutput);
    SmartDashboard.putNumber("pivot Setpoint", pivotPID.getSetpoint());
    SmartDashboard.putNumber("pivotabsolute", pivotAbsoluteEncoder.getPosition());

    // pivotOutput = pivotPID.calculate(pivotAngle) + Math.toDegrees(pivotFF.calculate (Math.toRadians(pivotAbsoluteEncoder.getPosition()), PIDs.AlgaeIntake.maxVelocity));
    // pivotMotor.set(pivotOutput);
  }

  public void RunIntakeIn(double speed) {
      algaeIntakeMotor.set(speed);
  }
  
  public void RunIntakeOut(double speed) {
    pivotMotor.set(-speed);
  }

  public void StopIntake() {
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

  private Boolean CanMoveIntakeIn(){
    return pivotAngle > RobotLimits.AlgaeIntake.minAngle;
  }
  private Boolean CanMoveIntakeOut(){
    return pivotAngle < RobotLimits.AlgaeIntake.maxAngle;
  }
  public void SetIntakeAngle(double angle)
  {
    if (angle > pivotPID.getSetpoint() && CanMoveIntakeOut()){
      pivotPID.setSetpoint(angle);
    }
    if (angle < pivotPID.getSetpoint() && CanMoveIntakeIn()){
      pivotPID.setSetpoint(angle);
    }
  }
  public double GetIntakeAngle() {
    return pivotAngle;
  }
  


  private void ReadSensorValues() {
    algaePresent = algaePresence.get();
    pivotAngle = pivotAbsoluteEncoder.getPosition();
  }

  // SysID nonsense

  public final Trigger atPivotMin = new Trigger(() -> !CanMoveIntakeIn());
  public final Trigger atPivotMax = new Trigger(() -> !CanMoveIntakeOut());

    private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  private final MutAngle          m_rotations      = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);
  private final MutAngularVelocity m_angularvelocity = DegreesPerSecond.mutable(0);

  private final SysIdRoutine PivotSysID = new SysIdRoutine
  (new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism(
   (voltage) -> this.sysidPivotRunVoltage(voltage.in(Volts)),
    log -> {
      DogLog.log("SysID/Algae/VoltageApplied", pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
      DogLog.log("SysID/Algae/Rotation", m_rotations.mut_replace(pivotAbsoluteEncoder.getPosition(),Degrees).in(Degrees));
      DogLog.log("SysID/Algae/Velocity", m_angularvelocity.mut_replace(pivotAbsoluteEncoder.getVelocity(),Degrees.per(Minute)).in(DegreesPerSecond));
    },
    this));

    public void sysidPivotRunVoltage(double V)
    {
      pivotMotor.setVoltage(V);
    }

  public Command sysIDPivotAll(){
    return PivotSysID.quasistatic(Direction.kForward).until(atPivotMin)
        .andThen(PivotSysID.quasistatic(Direction.kReverse).until(atPivotMax))
        .andThen(PivotSysID.dynamic(Direction.kForward).until(atPivotMin))
        .andThen(PivotSysID.dynamic(Direction.kReverse).until(atPivotMax))
        .andThen(Commands.print("DONE"));
  }
}
