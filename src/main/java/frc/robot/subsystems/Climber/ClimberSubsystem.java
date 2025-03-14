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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climberMotor;
  private SparkMax ClimberPivot;
  private SparkMaxConfig ClimberMotorConfig;
  private SparkMaxConfig ClimberPivotConfig;
  private DigitalInput cagePresenceSensor;
  private boolean cagePresent;

  private AbsoluteEncoder climberAngleEncoder;
  private double climberAngle;
  SparkMax CoralFlapRight = new SparkMax(CanIDs.CoralElevator.CoralFlapRight, MotorType.kBrushless);
    SparkMaxConfig coralFlapRightConfig = new SparkMaxConfig();
  //PIDController ClimberPID = new PIDController(PIDs.Climber.kp, PIDs.Climber.ki, PIDs.Climber.kd);
  double climberOutput;
  boolean climberOut = false;
  boolean ClimberIdle = false;
  PIDController RightFlapPID = new PIDController(0.0075, 0, 0);
  AbsoluteEncoder CoralFlapRightEncoder = CoralFlapRight.getAbsoluteEncoder();
  //public final Trigger atClimberMin = new Trigger(() -> !CanMoveClimberIn());
 // public final Trigger atClimberMax = new Trigger(() -> !CanMoveClimberOut());

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SparkMax(CanIDs.Climber.Winch, MotorType.kBrushless);
    ClimberMotorConfig = new SparkMaxConfig();
    ClimberPivot = new SparkMax(CanIDs.Climber.Pivot, MotorType.kBrushless);
    ClimberPivotConfig = new SparkMaxConfig();
    cagePresenceSensor = new DigitalInput(DIO.Climber.CagePresence);
    climberAngleEncoder = climberMotor.getAbsoluteEncoder();
    
  
   // PIDController RightFlapPID = new PIDController(PIDs.CoralElevator.RightFlap.kp,PIDs.CoralElevator.RightFlap.ki,PIDs.CoralElevator.RightFlap.kd );
  RightFlapPID.setSetpoint(GetRightFlap());
  // LeftFlapPID.setSetpoint(GetLeftFlap());
   RightFlapPID.setTolerance(5);
   RightFlapPID.enableContinuousInput(0, 360);
  // LeftFlapPID.setTolerance(3);

  // LeftFlapPID.enableContinuousInput(0, 360);
   //CoralFlapLeftConfig.absoluteEncoder.inverted(true);
  // CoralFlapLeftConfig.inverted(true);
   coralFlapRightConfig.inverted(true);
  // CoralFlapLeftConfig.idleMode(IdleMode.kBrake);
   coralFlapRightConfig.idleMode(IdleMode.kBrake);
  // CoralFlapLeft.configure(CoralFlapLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
   CoralFlapRight.configure(coralFlapRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    ReadSensorValues();
    double rightFlapOutput = RightFlapPID.calculate(GetRightFlap());
    SmartDashboard.putNumber("RightFlap", GetRightFlap());
    SmartDashboard.putNumber("ClimberEncoder", climberAngleEncoder.getPosition());
      rightFlapOutput = -MathUtil.clamp(rightFlapOutput, -.1, .1);
    CoralFlapRight.set(rightFlapOutput); //move to climber SS and only PID if we are setting up for a climb
   // System.out.println(climberAngle);
    if (climberOut)
    {
      //System.out.println("Climber Out");
      if((climberAngle <160 || climberAngle> 300) && !ClimberIdle)
      {
        ClimberPivot.set(0.1);
        //System.out.println("Climber Spd 0.05");
      }
      else 
      {
        ClimberPivot.set(0);
      }
    }
      if (!climberOut)
    {
     // System.out.println("Climber In");
      if(climberAngle >10 && climberAngle <170 && !ClimberIdle)
      {
        ClimberPivot.set(-0.1);
        //System.out.println("Climber Spd -0.05");
      }
      else 
      {
        ClimberPivot.set(0);
      }
      
    }
    //climberOutput = ClimberPID.calculate(climberAngle);
    // climberMotor.set(climberOutput);
  }
  public void ClimberOut()
  {
    ClimberPivotConfig.idleMode(IdleMode.kBrake);  
    ClimberPivot.configure(ClimberPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    climberOut = true;
  }
  public void ClimberIn()
  {
    ClimberPivotConfig.idleMode(IdleMode.kCoast);
    ClimberPivot.configure(ClimberPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    climberOut = false;
  }
  
  public void StopClimber(){
    climberMotor.set(0);
    ClimberIdle = true;
  }
  public void RunClimberIn(double speed){
   // System.out.println("RunClimberIn");
    ClimberPivotConfig.idleMode(IdleMode.kCoast);
    ClimberPivot.configure(ClimberPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    climberMotor.set(-speed);
    ClimberIdle = true;
    climberOut = false;
  }
  public void RunClimberOut(double speed){
    climberMotor.set(speed);
    ClimberIdle = true;
  }

  private Boolean CanMoveClimberIn(){
    return climberAngle > RobotLimits.Climber.minAngle;
  }
  private Boolean CanMoveClimberOut(){
    return climberAngle < RobotLimits.Climber.maxAngle;
  }
  public void SetRightFlap(double Angle)
  {
    RightFlapPID.setSetpoint(Angle);
  }
  public double GetRightFlap()
  {
    return CoralFlapRightEncoder.getPosition();
  }
  


  private void ReadSensorValues() {
    climberAngle = climberAngleEncoder.getPosition();
    cagePresent = cagePresenceSensor.get();
  }

  public double GetClimberEncoder(){
    return climberAngleEncoder.getPosition();
  }



  // SysID nonsense

  // private final MutAngle m_rotations = Rotations.mutable(0);
  // private final MutAngularVelocity m_AngularVelocity = RotationsPerSecond.mutable(0);

  // public Command sysIDClimberAll(){
  //   return (ClimberSysID.dynamic(Direction.kForward).until(atClimberMax)
  //       .andThen(ClimberSysID.dynamic(Direction.kReverse).until(atClimberMin))
  //       .andThen(ClimberSysID.quasistatic(Direction.kForward).until(atClimberMax))
  //       .andThen(ClimberSysID.quasistatic(Direction.kReverse).until(atClimberMin))
  //       .andThen(Commands.print("DONE")));
  // }

//  private final SysIdRoutine ClimberSysID = new SysIdRoutine(
//    new SysIdRoutine.Config(),
//    new SysIdRoutine.Mechanism(
//    climberMotor::setVoltage,
//    log -> {
//      DogLog.log("SysID/Climber/VoltageApplied", climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
//      DogLog.log("SysID/Climber/Position", m_rotations.mut_replace(climberAngleEncoder.getPosition(),Degrees).in(Degrees));
//      DogLog.log("SysID/Climber/Velocity", m_AngularVelocity.mut_replace(climberAngleEncoder.getVelocity(),Degrees.per(Minute)).in(DegreesPerSecond));
//
//    },
//    this));
  }