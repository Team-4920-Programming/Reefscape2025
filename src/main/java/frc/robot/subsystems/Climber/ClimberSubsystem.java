// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import dev.doglog.DogLog;

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

  public final Trigger atClimberMin = new Trigger(() -> !CanMoveClimberIn());
  public final Trigger atClimberMax = new Trigger(() -> !CanMoveClimberOut());

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

  private Boolean CanMoveClimberIn(){
    return climberAngle > RobotLimits.Climber.minAngle;
  }
  private Boolean CanMoveClimberOut(){
    return climberAngle < RobotLimits.Climber.maxAngle;
  }
  public void SetClimberAngle(double angle)
  {
    if (angle > ClimberPID.getSetpoint() && CanMoveClimberIn()){
      ClimberPID.setSetpoint(angle);
    }
    if (angle < ClimberPID.getSetpoint() && CanMoveClimberOut()){
      ClimberPID.setSetpoint(angle);
    }
  }
  


  private void ReadSensorValues() {
    climberAngle = climberAngleEncoder.getPosition();
    cagePresent = cagePresenceSensor.get();
  }



  // SysID nonsense

  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutAngularVelocity m_AngularVelocity = RotationsPerSecond.mutable(0);

  public Command sysIDClimberAll(){
    return (ClimberSysID.dynamic(Direction.kForward).until(atClimberMax)
        .andThen(ClimberSysID.dynamic(Direction.kReverse).until(atClimberMin))
        .andThen(ClimberSysID.quasistatic(Direction.kForward).until(atClimberMax))
        .andThen(ClimberSysID.quasistatic(Direction.kReverse).until(atClimberMin))
        .andThen(Commands.print("DONE")));
  }

    private final SysIdRoutine ClimberSysID = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
      climberMotor::setVoltage,
      log -> {
        DogLog.log("SysID/Climber/VoltageApplied", climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        DogLog.log("SysID/Climber/Position", m_rotations.mut_replace(climberAngleEncoder.getPosition(),Degrees).in(Degrees));
        DogLog.log("SysID/Climber/Velocity", m_AngularVelocity.mut_replace(climberAngleEncoder.getVelocity(),Degrees.per(Minute)).in(DegreesPerSecond));

      },
      this));

  
}
