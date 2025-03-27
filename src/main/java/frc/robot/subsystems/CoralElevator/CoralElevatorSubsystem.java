// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.ThreadPoolExecutor.AbortPolicy;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.prefs.NodeChangeEvent;

import javax.lang.model.util.ElementScanner14;

import org.dyn4j.geometry.decompose.EarClipping;
import org.opencv.core.Mat;

import com.fasterxml.jackson.databind.BeanProperty;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.BoltLog;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs.CoralElevator;
import frc.robot.Constants.PIDs.CoralElevator.Elevator;
import frc.robot.Constants.PIDs.CoralElevator.LeftFlap;
import frc.robot.Constants.PIDs.CoralElevator.RightFlap;
import frc.robot.Constants.PIDs.CoralElevator.TestElevator;
import frc.robot.Constants.PIDs.CoralElevator.Wrist;
import frc.robot.Constants.RobotPositions.AlgaeL2;
import frc.robot.Constants.RobotPositions.AlgaeL3;
import frc.robot.Constants.RobotPositions.CoralStation;
import frc.robot.Constants.RobotPositions.Level1;
import frc.robot.Constants.RobotPositions.Level2;
import frc.robot.Constants.RobotPositions.Level3;
import frc.robot.Constants.RobotPositions.Level4;
import frc.robot.Constants.RobotPositions.SafeTransition;
import frc.robot.Constants.RobotPositions.StartingConfig;
import frc.robot.Constants.RobotPositions.TransportCoralDown;
import frc.robot.Constants.RobotPositions.TransportCoralUp;
import frc.robot.TrajectoryPlanner4920.MechState;
import frc.robot.TrajectoryPlanner4920.Node;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Constants.RobotMotionLimits;
import frc.robot.Constants.RobotPositions;
import frc.robot.Robot;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import dev.doglog.DogLog;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;

import frc.robot.TrajectoryPlanner4920;

public class CoralElevatorSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final BoltLog BoltLogger = new BoltLog();
  private Mechanism2d SimElevator = new Mechanism2d(1.5, 3);
  private MechanismRoot2d SimElevatorRoot;
  private MechanismLigament2d SimStage1;
  private MechanismLigament2d SimElbow;
  private MechanismLigament2d SimWrist;
  
  // DataHighway
  public boolean DH_Out_HasCoral = false;
  public double DH_In_DistanceFromReef = 0;
  public boolean DH_In_RedZone = true;
  public boolean DH_In_YellowZone = false;
  public boolean DH_In_CoralZone = false;
  public Pose2d DH_In_RobotPose = new Pose2d();
  public boolean OverrideRedZone = false; //set from the algae commands
  public int DH_Out_ScoreSelection = 1;
  public boolean DH_SimIn_HasCoral = false;

  /** Physical Robot Init START**/
  
  // Motors

  SparkFlex ElevatorStageMotor = new SparkFlex(CanIDs.CoralElevator.ElevatorStage, MotorType.kBrushless);
  SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  SparkFlex ElbowMotor = new SparkFlex(CanIDs.CoralElevator.Elbow, MotorType.kBrushless);
  SparkFlexConfig elbowConfig = new SparkFlexConfig();
  SparkMax WristMotor = new SparkMax(CanIDs.CoralElevator.Wrist, MotorType.kBrushless);
  SparkMaxConfig wristConfig = new SparkMaxConfig();
  SparkMax CoralIntakeMotor = new SparkMax(CanIDs.CoralElevator.CoralIntake, MotorType.kBrushless);
  SparkMaxConfig CoralIntakeConfig = new SparkMaxConfig();
 // SparkMax CoralFlapLeft = new SparkMax(CanIDs.CoralElevator.CoralFlapLeft, MotorType.kBrushless);

 // SparkMaxConfig CoralFlapLeftConfig = new SparkMaxConfig();
  

  // PIDs

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(0.25, 0.25);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(PIDs.CoralElevator.TestElevator.kp, 0, PIDs.CoralElevator.TestElevator.kd, m_constraints, Constants.LOOP_TIME);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(PIDs.CoralElevator.TestElevator.ks, PIDs.CoralElevator.TestElevator.kg, PIDs.CoralElevator.TestElevator.kv);
  
  private final TrapezoidProfile.Constraints m_elbowconstraints =
      new TrapezoidProfile.Constraints(720, 720);
  private final ProfiledPIDController m_elbowcontroller =
  new ProfiledPIDController(PIDs.CoralElevator.TestElbow.kp, 0, PIDs.CoralElevator.TestElbow.kd, m_elbowconstraints, Constants.LOOP_TIME);
private final ArmFeedforward elbowFF = new ArmFeedforward(PIDs.CoralElevator.TestElbow.ks, PIDs.CoralElevator.TestElbow.kg, PIDs.CoralElevator.TestElbow.kv);


private final TrapezoidProfile.Constraints m_wristConstraints =
      new TrapezoidProfile.Constraints(180, 60);
  private final ProfiledPIDController m_wristcontroller =
  new ProfiledPIDController(PIDs.CoralElevator.TestWrist.kp, 0, PIDs.CoralElevator.TestWrist.kd, m_wristConstraints, Constants.LOOP_TIME);
private final ArmFeedforward wristFF = new ArmFeedforward(PIDs.CoralElevator.TestWrist.ks, PIDs.CoralElevator.TestWrist.kg, PIDs.CoralElevator.TestWrist.kv);

  
  PIDController ElevatorPID = new PIDController(PIDs.CoralElevator.TestElevator.kp, PIDs.CoralElevator.TestElevator.ki, PIDs.CoralElevator.TestElevator.kd);
  ElevatorFeedforward EFF = new ElevatorFeedforward(PIDs.CoralElevator.TestElevator.ks,PIDs.CoralElevator.TestElevator.kg,PIDs.CoralElevator.TestElevator.kv);
  // PIDController ElevatorPID = new PIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd);
  PIDController ElbowPID = new PIDController(PIDs.CoralElevator.Elbow.kp, PIDs.CoralElevator.Elbow.ki, PIDs.CoralElevator.Elbow.kd);
  PIDController WristPID = new PIDController(PIDs.CoralElevator.Wrist.kp, PIDs.CoralElevator.Wrist.ki, PIDs.CoralElevator.Wrist.kd);
  
  // Limit Switch
  DigitalInput ElevatorUpStop = new DigitalInput(DIO.CoralElevator.UpStop);
  DigitalInput ElevatorDownStop = new DigitalInput(DIO.CoralElevator.DownStop);
  DigitalInput ElevatorCoralPresence = new DigitalInput(DIO.CoralElevator.CoralPresence);
  Ultrasonic DistanceSensor = new Ultrasonic(5, 6);
  //DigitalInput UltrasonicTrigger = new DigitalInput(DIO.CoralElevator.UltrasonicTrigger);

  // Encoders
  Encoder ElevEnc = new Encoder(0, 4, false);
  

  RelativeEncoder ElevatorEncoder = ElevatorStageMotor.getEncoder();
  AbsoluteEncoder ElbowAbsoluteEncoder = ElbowMotor.getAbsoluteEncoder();
  RelativeEncoder ElbowRelativeEncoder = ElbowMotor.getEncoder();
  AbsoluteEncoder WristAbsoluteEncoder = WristMotor.getAbsoluteEncoder();
  RelativeEncoder WristRelative = WristMotor.getEncoder();
 
  // LaserCan
  private LaserCan LaserCan = new LaserCan(CanIDs.Sensor.LaserCAN);
  int elevatorHeightMM = 15;

  //Elevator Low Pass Filter
  LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.03, 0.02);
  double filteredelevatorHeight = 0.15;

  //MotorOutputs

  double elevatorOutput;
  double elbowOutput;
  double wristOutput;
  boolean SetpointsFrozen = false;
  boolean PabloOverride = true;
  boolean justScored = false;
  boolean isScoring = false;

  /** Physical Robot Init END**/

  double tmpElbowSetpointHolder = 999;
  double tmpElevatorSetpointHolder = 999;
  double tmpWristSetpointHolder = 999;

  double tmpMotionProfileElbowHolder = 999;
  double tmpMotionProfileWristHolder = 999;


//Mike's new logic for Setpoints - March 22
  double ElevatorGoal = 0.0;
  double WristGoal = 0.0;
  double ElbowGoal = 0.0;
  boolean SawElbowGoal = false;
  boolean SawWristGoal = false;
  boolean SawElevatorGoal = false;

  /** SIM Robot Init START */
  DCMotor ElevatorStageGearbox = DCMotor.getNEO(1);
  DCMotor ElbowGearbox = DCMotor.getNEO(1);
  DCMotor WristGearbox = DCMotor.getNEO(1);
  DCMotor CoralIntakeGearbox = DCMotor.getNEO(1);

  SparkSim ElevatorUpDownMotorSim = new SparkSim(ElevatorStageMotor, ElevatorStageGearbox);
  SparkSim ElbowMotorSim = new SparkSim(ElbowMotor, ElbowGearbox);
  SparkMaxSim WristMotorSim = new SparkMaxSim(WristMotor, WristGearbox);
  SparkMaxSim CoralIntakeSim = new SparkMaxSim(CoralIntakeMotor, CoralIntakeGearbox);
  double SimElevatorHeight = 0;
  double SimElbowAngle = 0;
  double SimWristAngle =0;
 
  // simulated motors were here

  /** SIM Robot Init END */

  // states


  private int ScoreSelection = 4; //default score level is 4

  public final Trigger atElevatorMin = new Trigger(() -> getDownStop());
  public final Trigger atElevatorMax = new Trigger(() -> getUpStop());
  public final Trigger atElbowMin = new Trigger(() -> GetElbowAngle() <= -10);
  public final Trigger atElbowMax = new Trigger(() -> GetElbowAngle() >= 220);
  public final Trigger atWristMin = new Trigger(() -> GetWristAngleWorldCoordinates() <= -50);
  public final Trigger atWristMax = new Trigger(() -> GetWristAngleWorldCoordinates() >= 210);
  
  public CoralElevatorSubsystem() {


    ElevEnc.setDistancePerPulse(Units.inchesToMeters(31.25) / (10432));
    ElevatorPID.setTolerance(0.015);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.inverted(true);
    elevatorConfig.smartCurrentLimit(80);
    // elevatorConfig.openLoopRampRate(0.1);
    elevatorConfig.voltageCompensation(12);
    ElevatorStageMotor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    ElbowPID.setTolerance(3);
 
    elbowConfig.idleMode(IdleMode.kBrake);
    elbowConfig.inverted(false);
    elbowConfig.absoluteEncoder.positionConversionFactor(360);
    elbowConfig.absoluteEncoder.velocityConversionFactor(6);
    elbowConfig.absoluteEncoder.inverted(true);
    elbowConfig.smartCurrentLimit(40);
    // elbowConfig.openLoopRampRate(0.25);
    ElbowMotor.configure(elbowConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.inverted(false);
    wristConfig.absoluteEncoder.positionConversionFactor(360);
    wristConfig.absoluteEncoder.velocityConversionFactor(6);
    wristConfig.absoluteEncoder.inverted(false);
    wristConfig.smartCurrentLimit(20);
    WristMotor.configure(wristConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    //   360/468.75 (1 revolation on the Elbow = 430 Revolutions of the Motor) 
    // 54/16 = 3.375
    // 125:1 Gearbox
    // every 125 rotations of the motor = 1 rotation of the 16 tooth pulley
    // every 3.375 rotatios of the 16 tooth pulley = 1 rotation of the elbow
    // 421.875 rotations of the motor is one rotation of the wrist

    // 54/16 = 3.375
    // 45:1 Gearbox
    // every 45 rotations of the motor = 1 rotation of the 16 tooth pulley
    // every 3.375 rotatios of the 16 tooth pulley = 1 rotation of the elbow
    // 151.875 rotations of the motor is one rotation of the wrist
    // 360/151.875 is conversion factor


    WristPID.setTolerance(3);
    // if robot.issimulation was here
    ElbowPID.setSetpoint(GetElbowAngle());
    ElevatorPID.setSetpoint(Math.max(getFilteredElevatorHeight(),0.16));
    CoralIntakeConfig.inverted(true);
    CoralIntakeMotor.configure(CoralIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    WristPID.setSetpoint(GetWristAngleWorldCoordinates());
    
    m_elbowcontroller.setTolerance(2);
    m_controller.setTolerance(0.015);
    m_wristcontroller.setTolerance(2);

    

    ScoreSelection = 4; //default to level 4
    if (Robot.isSimulation())
    {
      setupSim();
    }

  }


  public void SetScoreSelection(int level)
  {
    ScoreSelection = level;
  
  }
  public int GetScoreSelection()
  {
    return ScoreSelection;
  
  }

  /* Elevator */

  public void SetElevatorPosition(double height)
  {
    // if (!SetpointsFrozen || (SetpointsFrozen && OverrideRedZone)){
    //   ElevatorPID.setSetpoint(height);
    //   tmpElevatorSetpointHolder = 999;
    // }
    // else{
    //   tmpElevatorSetpointHolder = height;
    // }
    if (ElevatorGoal != height && height <= RobotLimits.Elevator.maxHeight)
    {
      //if (height < 0.7)
      ElevatorGoal = height;
      SawElevatorGoal = false;
    }

    m_controller.setGoal(new State(height, 0.0));
    
  }

  public boolean IsElevatorAtSetpoint(double target){
    boolean b1 = Math.abs(getFilteredElevatorHeight() - target) <= ElevatorPID.getErrorTolerance();
    boolean b2 = ElevatorPID.getSetpoint() > getFilteredElevatorHeight() && getUpStop();
    boolean b3 = ElevatorPID.getSetpoint() < getFilteredElevatorHeight() && getDownStop();
    boolean b4 = ElevatorStageMotor.get() > 0 && getFilteredElevatorHeight() >= RobotLimits.Elevator.maxHeight;
    boolean b5 = ElevatorStageMotor.get() < 0 && getFilteredElevatorHeight() <= RobotLimits.Elevator.minHeight;
    return b1 || b2 || b3 || b4 || b5;
  }
  public boolean IsWristAtSetpoint(double target){
    return Math.abs(GetWristAngleWorldCoordinates() - target) <= WristPID.getErrorTolerance();
  }
  public boolean IsElbowAtSetpoint(double target){
    return Math.abs(GetElbowAngle() - target) <= ElbowPID.getErrorTolerance();
  }
  public BooleanSupplier test(){
    BooleanSupplier b = () -> true;
      return b;
    };

  private Boolean ElevatorClearToMoveDown(){
    double wristAngle = GetWristAngleWorldCoordinates();
    double elbowAngle = GetElbowAngle();
    double height = getFilteredElevatorHeight();
    boolean hascoral = isCoralPresent();
    boolean EClear = false;

      if (height <= 0.275){
        EClear = true;
      }
      return EClear;
  }
  private Boolean ElevatorClearToMoveUp(){
    double wristAngle = GetWristAngleWorldCoordinates();
    double elbowAngle = GetElbowAngle();
    double height = getFilteredElevatorHeight();
    boolean hascoral = isCoralPresent();
    boolean EClear = false;

      if (height >= 0.43 || (height <= 0.2 && !hascoral)){
        EClear = true;
      }
      return EClear;
  }
  
  private Boolean ElevatorClearToMoveCheck(){
    Boolean elbowClear = false;
    boolean wristClear = false;
    // if (GetElbowAngle() > RobotMotionLimits.Elbow.minAngle && GetElbowAngle() < RobotMotionLimits.Elbow.maxAngle )
    // {
    //   elbowClear = true;
    // }

    // //wrist > 90 0.36 cant go up
    // //0.55 cant go down
    double wristAngle = GetWristAngleWorldCoordinates();
    double elbowAngle = GetElbowAngle();
    // if (wristAngle > RobotMotionLimits.Wrist.minAngle  && GetWristAngleWorldCoordinates() < 180 )
    // {
    //   wristClear = true;
    // }

    if (elbowAngle >= 35)
    {
      elbowClear = true;
      wristClear = true;
    }
    else{
      if (wristAngle >= 80 && wristAngle <= 100){
        wristClear = true;
      }
      else if (elbowAngle >= 10 && wristAngle <= 95 && wristAngle >= -55){
        wristClear = true;
      }
      if (elbowAngle >= 10){
        elbowClear = true;
      }
      if (elbowAngle >= -2)
        elbowClear = true;
        if(wristAngle >= 70 && wristAngle <= 95){
          wristClear = true;
        }
    }


    return elbowClear && wristClear;
  }
  
  private Boolean CanMoveElevatorUp(){
    return (!getUpStop() && (ElevatorClearToMoveCheck() || ElevatorClearToMoveUp())); //&& !isWristPassingThroughVertical());
  }

  private Boolean CanMoveElevatorDown(){
    return !getDownStop() && (ElevatorClearToMoveCheck() || ElevatorClearToMoveDown());// && !isWristPassingThroughVertical());
  }

  /* Elbow */

  public void SetElbowAngle(double angle)
  {
    //if (!SetpointsFrozen || (SetpointsFrozen && OverrideRedZone)){
    //   if (tmpElbowSetpointHolder == 999){
    //     //ElbowPID.setSetpoint(angle);
    //   }
    //   else{
    //     ElbowPID.setSetpoint(tmpElbowSetpointHolder);
    //   }
      
    //   tmpElbowSetpointHolder = 999;
    // }
    // else{
    //   tmpElbowSetpointHolder = angle;
    // }
    
    if (ElbowGoal != angle)
    {
      ElbowGoal = angle;
      SawElbowGoal = false;
      System.out.println("ElbowGoal Set "+ angle);
    }

    m_elbowcontroller.setGoal(new State(angle, 0.0));


    
  }
  
  private Boolean CanMoveElbowInc(){
    boolean elevatorClear = false;
    boolean wristClear = false;
    if (getFilteredElevatorHeight() > RobotMotionLimits.Elevator.minHeight && getFilteredElevatorHeight() < RobotMotionLimits.Elevator.maxHeight)
    {
        elevatorClear = true;
    }
    if (GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle + 10 && GetWristAngleWorldCoordinates() < RobotMotionLimits.Wrist.maxAngle - 5)
    {
     
        wristClear = true;
      
    }
    return elevatorClear && wristClear;
  }

  private Boolean CanMoveElbowDec(){

    boolean elevatorClear = false;
    boolean wristClear = false;
    if (getFilteredElevatorHeight() > RobotMotionLimits.Elevator.minHeight && getFilteredElevatorHeight() < RobotMotionLimits.Elevator.maxHeight)
    {
        elevatorClear = true;
    }

    if (GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle + 10)
    {
     
        wristClear = true;
      
    }

    return elevatorClear && wristClear;

  }

  public Double GetElbowAngle() {
    //Angle Offset of 25 Degree when zero against inside hardstop
    double pos = 0;
    if (Robot.isReal()){
      pos = ElbowAbsoluteEncoder.getPosition(); 
      pos -= 90;
    }
    else
    {
      pos = SimElbowAngle;
    }
    
    return pos;
    
  }

  /* Wrist */

  public void SetWristAngle(double angle)
  { 
    if (!SetpointsFrozen || (SetpointsFrozen && OverrideRedZone)){
      if (tmpWristSetpointHolder == 999){
        //WristPID.setSetpoint(angle);
      }
      else{
        WristPID.setSetpoint(tmpWristSetpointHolder);
      }
      tmpWristSetpointHolder = 999; 
    }
    else{
      tmpWristSetpointHolder = angle;
    }
    if (angle != WristGoal){
      WristGoal = angle;
      SawWristGoal = false;
    }

    m_wristcontroller.setGoal(new State(angle, 0));
    }
    

  private Boolean CanMoveWristInc(){
    return GetWristAngleWorldCoordinates() < RobotMotionLimits.Wrist.maxAngle;
  }
  
  private Boolean CanMoveWristDec(){
    return GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle;
  }

  public double GetWristAngle(){
    if (Robot.isReal())
      return WristAbsoluteEncoder.getPosition();
    else
    {
      return SimWristAngle;
    }
  
  }

  public double GetWristAngleWorldCoordinates(){
    double wAng = GetWristAngle();
    double eAng = GetElbowAngle();

    double worldAngle = 90-wAng + eAng;

    if (worldAngle <= -95){
      return worldAngle + 360;
    }
    return  worldAngle;
  }

  public void setIntakeSpeed(double speed)
  {
    CoralIntakeMotor.set(speed);
  }

  public boolean isIntakeRunning() {
    return CoralIntakeMotor.get() > 0;
  }

  public void MatchSetup(){
      if(!DH_In_RedZone){
      ElevatorPID.setSetpoint(CoralStation.height);
      }
      else{
      ElevatorPID.setSetpoint(getFilteredElevatorHeight());
      }
    }

  public void setArmPosition(double height, double elbow, double wrist){
      SetElevatorPosition(height);
      SetElbowAngle(elbow);
      SetWristAngle(wrist);
  }

  public double getFilteredElevatorHeight(){

    return filteredelevatorHeight;
  }

  public double getUltrasonic(){
      return DistanceSensor.getRangeInches();
  }
  public void setPabloOverride(boolean b){
    PabloOverride = b;
  }
  public boolean getPableOverride(){
    return PabloOverride;
  }

  public void setIsScoring(boolean b){
    isScoring = b;
  }
  public boolean getIsScoring(){
    return isScoring;
  }

  public void setJustScored(boolean b){
    justScored = b;
  }
  public boolean getJustScored(){
    return justScored;
  }
  public boolean isElevatorAtGoal()
  {
    double curElevPos = getFilteredElevatorHeight();
    double curElevTol = ElevatorPID.getErrorTolerance();
    boolean atGoal = (Math.abs(curElevPos - ElevatorGoal) < curElevTol);
    if (atGoal)
      {
        SawElevatorGoal = true;
      }
    return atGoal;
  }
  public boolean isElbowAtGoal()
  {
    double curElbowPos = GetElbowAngle();
    double curElbowTol = ElbowPID.getErrorTolerance();
    boolean atGoal =  (Math.abs(curElbowPos - ElbowGoal) < curElbowTol);
    if (atGoal)
      {
        SawElbowGoal = true;
      }
    return atGoal;
  }
  public boolean isWristAtGoal()
  {
    double curWristPos = GetWristAngleWorldCoordinates();
    double curWristTol = WristPID.getErrorTolerance();
    boolean atGoal =   (Math.abs(curWristPos - WristGoal) < curWristTol);
    if (atGoal)
      {
        SawWristGoal = true;
      }
    return atGoal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // resetEncoderCount();
    double test1 = m_elbowcontroller.calculate(GetElbowAngle(), m_elbowcontroller.getGoal().position);
    double test2 = elbowFF.calculate(Units.degreesToRadians(90.699), m_elbowcontroller.getSetpoint().velocity);
    double test3 = m_wristcontroller.calculate(GetWristAngleWorldCoordinates(), m_wristcontroller.getGoal().position);
    double test4 = wristFF.calculate(Units.degreesToRadians(-90 - GetElbowAngle()), m_wristcontroller.getSetpoint().velocity);
    DistanceSensor.setAutomaticMode(true);
    DistanceSensor.setEnabled(true);
    if (DistanceSensor.isRangeValid())
    {
      DogLog.log("CoralElevatorSS/UltasonicDistanceInches", DistanceSensor.getRangeInches());
    }
    if (Robot.isReal())
    {
      filteredelevatorHeight = elevatorFilter.calculate(getHeightLaserMeters());
    }
    else
    {
      filteredelevatorHeight = SimElevatorHeight;
    }
    if (DH_In_RedZone && !OverrideRedZone  && !SetpointsFrozen){
      
      //Freeze PIDs at current positon
      FreezeSetpoints(false,false,false);

    }
    if (!DH_In_RedZone && SetpointsFrozen){

      UnfreezeSetPoints();

    }
    if (DH_In_RedZone && OverrideRedZone && SetpointsFrozen){
      
      UnfreezeSetPoints();

    }

    // double test = ElevatorPID.calculate(getElevDistance(),ElevatorGoal);
    // ElevatorStageMotor.setVoltage(
    //     m_controller.calculate(getElevDistance())
    //         + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    // ElevatorStageMotor.set(test + EFF.calculate(0));

    // ElbowMotor.setVoltage(test1 + test2); 

    WristMotor.setVoltage(test3);

    
/****************** LOOK AT *************************
 */
    // if ((isElevatorPassingThroughRedZone() || isElevatorInRedZone()) && WristPID.getSetpoint() >= 90){
    //     if (tmpWristSetpointHolder == 999){
    //       tmpWristSetpointHolder = WristPID.getSetpoint();
    //       WristPID.setSetpoint(90);
    //     }
    //   }
    //   else if (tmpWristSetpointHolder != 999){
    //     WristPID.setSetpoint(tmpWristSetpointHolder);
    //     tmpWristSetpointHolder = 999 ;
    //   }
 /*
*****************************************************/
    
    //filteredelevatorHeight = elevatorFilter.calculate(getHeightLaserMeters()); - Duplicated above MJD MArch 22
    double elevatorPIDValue = ElevatorPID.calculate(filteredelevatorHeight);
    double linearVelocity = getVelocityMetersPerSecond();
    double elevatorFFValue = 0.6;
    double elbowOutputFF=0;
    if (DriverStation.isEnabled()){
      //elevatorOutput = MathUtil.clamp(elevatorPIDValue,-1,1);
      elbowOutput = ElbowPID.calculate(GetElbowAngle());
      elbowOutput = -elbowOutput;
      elbowOutput = MathUtil.clamp(elbowOutput, -1.0, 1.0);//was -.5 and .5
      // if (ElbowPID.getSetpoint() > GetElbowAngle())
        // elbowOutputFF = Math.abs(Math.sin(Units.degreesToRadians(GetElbowAngle())))* 2.0;
      // else
        // elbowOutputFF = Math.abs(Math.sin(Units.degreesToRadians(GetElbowAngle())))*.8;
      // elbowOutput = elbowOutput * Math.max(elbowOutputFF,0.15);
      double elbowMinSpeed = 0.02;
      if (GetElbowAngle() >= -15 && GetElbowAngle() <= 15 && elbowOutput < 0){
        elbowMinSpeed = 0.02;
    }
    // if (elbowOutput < 0){
    //   elbowOutput = Math.min(elbowOutput, -elbowMinSpeed); //wasn't assigned back to the elbow output
    // }
    // if (elbowOutput > 0){
    //   elbowOutput = Math.max(elbowOutput, elbowMinSpeed);
    // }
      if (ElbowPID.atSetpoint())
      {
          elbowOutput = 0;
      }
      wristOutput = WristPID.calculate(GetWristAngleWorldCoordinates());
    }
    DogLog.log("CoralElevatorSS/Elevator/EncDistance", getElevDistance());
    DogLog.log("CoralElevatorSS/Elevator/EncVelocity", getVelocityMetersPerSecond());
    DogLog.log("CoralElevatorSS/Elevator/EPID", m_controller.calculate(getElevDistance()));
    DogLog.log("CoralElevatorSS/Elevator/Ectrlpgoal", m_controller.getGoal().position);
    DogLog.log("CoralElevatorSS/Elevator/Ectrlvgoal", m_controller.getGoal().velocity);
    DogLog.log ("CoralElevatorSS/Elbow/NewFF", test2);

    DogLog.log ("CoralElevatorSS/Elbow/NewPID", test1);
    DogLog.log ("CoralElevatorSS/Elbow/newPIDPError", m_elbowcontroller.getPositionError());
    DogLog.log ("CoralElevatorSS/Elbow/newPIDVError", m_elbowcontroller.getVelocityError());

    DogLog.log ("CoralElevatorSS/Wrist/NewFF", test4);

    DogLog.log ("CoralElevatorSS/Wrist/NewPID", test3);
    DogLog.log ("CoralElevatorSS/Wrist/newPIDPError", m_wristcontroller.getPositionError());
    DogLog.log ("CoralElevatorSS/Wrist/newPIDVError", m_wristcontroller.getVelocityError());


    DogLog.log("CoralElevatorSS/Elevator/EFF", m_feedforward.calculate(m_controller.getSetpoint().velocity));
    
    DogLog.log("CoralElevatorSS/Elevator/ThroughBoreRelative", ElevEnc.get());
    DogLog.log ("CoralElevatorSS/Elbow/FF", elbowOutputFF);
    DogLog.log("CoralElevatorSS/Elevator/ElevatorPIDOutput", elevatorPIDValue);
    DogLog.log("CoralElevatorSS/Elevator/ElevatorPIDSetpoint", ElevatorPID.getSetpoint());
    DogLog.log("CoralElevatorSS/Elevator/ElevatorPIDError", ElevatorPID.getError());
    DogLog.log("CoralElevatorSS/Elevator/ElevatorHeight", getFilteredElevatorHeight());
    DogLog.log("CoralElevatorSS/Elevator/RawElevatorHeight", getHeightLaserMeters());
    DogLog.log("CoralElevatorSS/Elevator/ElevatorGoal", ElevatorGoal);

    //Elbow Data
    DogLog.log("CoralElevatorSS/Elbow/ElbowAngle",GetElbowAngle());
    DogLog.log("CoralElevatorSS/Elbow/ElbowPIDSetpoint",ElbowPID.getSetpoint());
    DogLog.log("CoralElevatorSS/Elbow/ElbowPIDError",ElbowPID.getError());
    DogLog.log("CoralElevatorSS/Elbow/ElbowPIDOutput",elbowOutput);
    DogLog.log("CoralElevatorSS/Elbow/CanElbowInc", CanMoveElbowInc());
    DogLog.log("CoralElevatorSS/Elbow/CanElbowDec", CanMoveElbowDec());
    DogLog.log("CoralElevatorSS/Elbow/ElbowGoal", ElbowGoal);

    DogLog.log("CoralElevatorSS/Wrist/WristPIDOutput", wristOutput);
    DogLog.log("CoralElevatorSS/Wrist/WristPIDSetpoint", WristPID.getSetpoint());
    DogLog.log("CoralElevatorSS/Wrist/WristAbsoluteAngle", WristAbsoluteEncoder.getPosition());
    DogLog.log("CoralElevatorSS/Wrist/WristPIDError", WristPID.getError());
    DogLog.log("CoralElevatorSS/Wrist/WristGoal", WristGoal);
    DogLog.log("CoralElevatorSS/Wrist/WristWorldAngle", GetWristAngleWorldCoordinates());
    DogLog.log("CoralElevatorSS/Wrist/CanWristInc", CanMoveWristInc());
    DogLog.log("CoralElevatorSS/Wrist/CanWirstDec", CanMoveWristDec());

    DogLog.log("CoralElevatorSS/Intake/HasCoral", isCoralPresent());
    DogLog.log("CoralElevatorSS/Intake/IntakeSpeed", CoralIntakeMotor.get());

    DogLog.log("CoralElevatorSS/Check/Elevator/CanMoveElevatorUp", CanMoveElevatorUp());
    DogLog.log("CoralElevatorSS/Check/Elevator/CanMoveElevatorDown", CanMoveElevatorDown());
    DogLog.log("CoralElevatorSS/Check/Elevator/ElevatorClearToMove",ElevatorClearToMoveCheck());
    DogLog.log("CoralElevatorSS/Check/Elevator/ElevatorClearToMoveUp", ElevatorClearToMoveUp());
    DogLog.log("CoralElevatorSS/Check/Elevator/ElevatorClearToMoveDown", ElevatorClearToMoveDown());

    DogLog.log("CoralElevatorSS/Check/Wrist/isWristPassingThroughVertical",isWristPassingThroughVertical());
    DogLog.log("CoralElevatorSS/Check/Elevator/isElevatorInRedZone", isElevatorInRedZone());
    DogLog.log("CoralElevatorSS/Check/Elevator/isElevatorPassingThroughRedZone", isElevatorPassingThroughRedZone());
    DogLog.log("CoralElevatorSS/Check/ScoreSelection", ScoreSelection);
    DogLog.log("CoralElevatorSS/Check/SetpointsFrozen", SetpointsFrozen);
    DogLog.log("CoralElevatorSS/Check/OverrideRedZone", OverrideRedZone);
    DogLog.log("CoralElevatorSS/Check/PabloOverride", PabloOverride);
    DogLog.log("CoralElevatorSS/Check/IsScoring", isScoring);
    DogLog.log("CoralElevatorSS/Check/JustScored", justScored);
    DogLog.log("CoralElevatorSS/Check/Elevator/ElevatorUpStop", getUpStop());
    DogLog.log("CoralElevatorSS/Check/Elevator/ElevatorDownStop", getDownStop());

    DogLog.log("CoralElevatorSS/Check/IsElevatorAtSetpoint", IsElevatorAtSetpoint(ElevatorPID.getSetpoint()));
    DogLog.log("CoralElevatorSS/Check/IsElbowAtSetpoint", IsElbowAtSetpoint(ElbowPID.getSetpoint()));
    DogLog.log("CoralElevatorSS/Check/IsWristAtSetpoint", IsWristAtSetpoint(WristPID.getSetpoint()));

    DogLog.log("CoralElevatorSS/Check/tmpElbow", tmpElbowSetpointHolder);
    DogLog.log("CoralElevatorSS/Check/tmpElevator", tmpElevatorSetpointHolder);
    DogLog.log("CoralElevatorSS/Check/tmpWrist", tmpWristSetpointHolder);
    DogLog.log("CoralElevatorSS/Check/tmpMotionElbow", tmpMotionProfileElbowHolder);
    DogLog.log("CoralElevatorSS/Check/tmpMotionWrist", tmpMotionProfileWristHolder);

    double  EmotorSpd =0;
    boolean atSet = true;
    double setpoint = 999;
    double EleFF = 0.0;
   if (getFilteredElevatorHeight() >= 0.5)
      EleFF = 0.03;
      else
      EleFF = 0.0;
//**********************************************
// Mike D Rework of Setpoint buffering - March 22
// If elevator needs to move - move Wrist/Elbow to safe first
// if  Wrist/Elbow are in safe position - move elevator
// if Elevator in position - move Wrist/Elbow
//******************************************* 
    boolean MoveElevator = false;
    if (SetpointsFrozen)
    {
      MoveElevator = true;
    }
    if (!SetpointsFrozen)
    {
      if (isWristAtGoal() && isElbowAtGoal())
        {
          MoveElevator = true;
         ElevatorPID.setSetpoint(ElevatorGoal);
        }
      if(ElevatorGoal < getFilteredElevatorHeight() && !DH_In_RedZone){
        MoveElevator = true;
        ElevatorPID.setSetpoint(ElevatorGoal);
      }
      if (isElevatorAtGoal() || (SawElevatorGoal && Math.abs(ElevatorPID.getError()) < 3* ElevatorPID.getErrorTolerance()))
      {
        WristPID.setSetpoint(WristGoal);
        ElbowPID.setSetpoint(ElbowGoal);
        MoveElevator = true;
      }
      else
      {
        WristPID.setSetpoint(RobotPositions.SafePosition.wrist);
        ElbowPID.setSetpoint(RobotPositions.SafePosition.elbow);
        //MoveElevator = false;
        boolean WristCloseEnough = Math.abs(WristPID.getError()) < 3* WristPID.getErrorTolerance();
        boolean ArmCloseEnought = (Math.abs(ElbowPID.getError()) < 4* ElbowPID.getErrorTolerance());
        if (WristCloseEnough && ArmCloseEnought && WristPID.getSetpoint() == RobotPositions.SafePosition.wrist && ElbowPID.getSetpoint() == RobotPositions.SafePosition.elbow){
          ElevatorPID.setSetpoint(ElevatorGoal);
          MoveElevator = true;
        }
      }
            
    }
    if (MoveElevator == false)
    {
      ElevatorPID.setSetpoint(getFilteredElevatorHeight());
    }
    if (MoveElevator)
    {
      //Elevator move requested - make sure we are not on a overtravel switch before moving
      //if ((elevatorOutput > 0 && !getUpStop()) || (elevatorOutput < 0 && !getDownStop()))
      //{
        if (getFilteredElevatorHeight() < ElevatorPID.getSetpoint()&& !getUpStop())
        {
         if (ElevatorPID.atSetpoint())
          elevatorOutput = 0;
        if (Math.abs(ElevatorPID.getError()) > ElevatorPID.getErrorTolerance())
          elevatorOutput = 0.20;
          if (Math.abs(ElevatorPID.getError()) > 0.2)
          elevatorOutput = 0.35;
            
        if (Math.abs(ElevatorPID.getError()) > 0.3)
          elevatorOutput = 0.5;
        if (Math.abs(ElevatorPID.getError()) > 0.5)
          elevatorOutput = .75;
          if (Math.abs(ElevatorPID.getError()) > 0.7)
          elevatorOutput = 1;
        
        }
        if (getFilteredElevatorHeight() > ElevatorPID.getSetpoint() && !getDownStop())
        {
        if (ElevatorPID.atSetpoint())
          elevatorOutput = 0;
        if (Math.abs(ElevatorPID.getError()) > ElevatorPID.getErrorTolerance())
          elevatorOutput = -0.1;
          if (Math.abs(ElevatorPID.getError()) > 0.15)
          elevatorOutput = -0.3;
          if (Math.abs(ElevatorPID.getError()) > 0.25)
          elevatorOutput = -0.5;
        if (Math.abs(ElevatorPID.getError()) > 0.5)
          elevatorOutput = -1;
        
        }
        //if (getFilteredElevatorHeight() > 0.7)
          //EleFF = 0.2;
//         double elevatorMinSpeed = 0.03;
//          if (elevatorOutput >0 && getFilteredElevatorHeight() < ElevatorGoal && ElevatorGoal < 0.3)
//           {
//             elevatorMinSpeed = 0.05;
//           }
//           if (elevatorOutput <0 && getFilteredElevatorHeight() > ElevatorGoal && ElevatorGoal < 0.3)
//           {
//             elevatorMinSpeed = 0.01;
//           }
// System.out.println("min speed"+elevatorMinSpeed);

//         if (getFilteredElevatorHeight() >= 0.5)
//           elevatorMinSpeed = 0.04;/// (RobotController.getBatteryVoltage()/13);
          
//         if (elevatorOutput < 0){

//           elevatorOutput = Math.min(elevatorOutput,-elevatorMinSpeed);
//         }
//         else{
//           elevatorOutput = Math.max(elevatorOutput, elevatorMinSpeed);
//         }
//       //}
        //////////////////UNCOMMENT: 
        // ElevatorStageMotor.set(elevatorOutput+ EleFF);
        // double test = ElevatorPID.calculate(filteredelevatorHeight);
        // ElevatorStageMotor.set(test + EFF.calculate(0));

      // }
      // else
      // {
      //   //On a overtravel don't move in requested direction
      // ElevatorStageMotor.set(0 + EleFF);
      
      // ElevatorStageMotor.set(0 + EFF.calculate((0)));

      // }
    }
    else
    {
      //not requested to move - so stop
     //////////////////UNCOMMENT:// 
    //  ElevatorStageMotor.set(0 + EleFF);
    // ElevatorStageMotor.set(0);

    } 
    // may need to add if at setpoint to stop moving.. hopefully the PID stops us.
    //UNCOMMENT
    // ElbowMotor.set(elbowOutput); 
    // WristMotor.set(wristOutput);

    DogLog.log("CoralElevatorSS/Elbow/ActualElbowMotorOutput", elbowOutput);
    DogLog.log("CoralElevatorSS/Wrist/ActualWristMotorOutput", wristOutput); 
    DogLog.log("CoralElevatorSS/Elevator/ClampedElevatorPIDOutput", elevatorOutput);
    
    DogLog.log("CoralElevatorSS/Elevator/MoveElevator", MoveElevator); 
    DogLog.log("CoralElevatorSS/Elevator/ElevatorAtGoal", isElevatorAtGoal()); 
    DogLog.log("CoralElevatorSS/Elbow/ElbowAtGoal", isElbowAtGoal()); 
    DogLog.log("CoralElevatorSS/Wrist/WristAtGoal", isWristAtGoal()); 
    

    if (Robot.isSimulation()){
      applyOutputToSim();
    }
//End of Mike's Changes
 
    //  if (tmpElevatorSetpointHolder == 999){
    //   atSet = IsElevatorAtSetpoint(ElevatorPID.getSetpoint());
    //   setpoint = ElevatorPID.getSetpoint();

    //  }
    //  else{
    //   atSet = IsElevatorAtSetpoint(tmpElevatorSetpointHolder);
    //   setpoint = tmpElevatorSetpointHolder;
    //  }
    //   if (!atSet)
    //   { 
    //     if ((CanMoveElevatorUp() && getFilteredElevatorHeight() < setpoint ) || (CanMoveElevatorDown() && getFilteredElevatorHeight() > setpoint)){
    //       double EleveFF = 0.025;
    //       if (tmpMotionProfileElbowHolder != 999){
    //         SetElbowAngle(tmpMotionProfileElbowHolder);
    //         tmpMotionProfileElbowHolder = 999;
    //       }
    //       if (tmpMotionProfileWristHolder != 999){
    //         SetWristAngle(tmpMotionProfileWristHolder);
    //         tmpMotionProfileWristHolder = 999;
    //       }
    //       // if (getFilteredElevatorHeight() > 0.30 && getFilteredElevatorHeight() < 0.5)
    //       //   EleveFF = 0.01;

    //       // EmotorSpd = elevatorOutput + EleveFF;
    //       // if (EmotorSpd < 0.05 && EmotorSpd>0)
    //       //   EmotorSpd = 0.05;
    //       // if (EmotorSpd > -0.05 && EmotorSpd<0)
    //       //   EmotorSpd = -0.05;  
    //       if (GetWristAngleWorldCoordinates() > 100 && isCoralPresent() && isElevatorPassingThroughRedZone())
    //       {
    //         ElevatorStageMotor.set(0 + EleFF);
    //       }
    //       else{
    //         // if (Math.abs(elevatorOutput) < 0.04){
    //         //   ElevatorStageMotor.set(0);
    //         // }
    //         // else{
    //       ElevatorStageMotor.set(elevatorOutput);
    //         // }
    //       } //10% speed for testing 
    //   }
    //   else if (((!CanMoveElevatorUp() && getFilteredElevatorHeight() < setpoint ) || (!CanMoveElevatorDown() && getFilteredElevatorHeight() > setpoint)) || isElevatorPassingThroughRedZone()){
    //     if (tmpMotionProfileWristHolder == 999){
    //       tmpMotionProfileWristHolder = WristPID.getSetpoint();
    //       SetWristAngle(90);
    //     }
    //     else{
    //       SetWristAngle(tmpMotionProfileElbowHolder);
    //     }
    //     if (tmpMotionProfileElbowHolder == 999){
    //       tmpMotionProfileElbowHolder = ElbowPID.getSetpoint();
    //       SetElbowAngle(15);  
    //     }
    //     else{
    //       SetElbowAngle(tmpMotionProfileElbowHolder);
    //     }
    //     ElevatorStageMotor.set(0 + EleFF);
    //   }
    // }
    // else{ 
    //   ElevatorStageMotor.set(0 + EleFF);

    //   if (tmpMotionProfileElbowHolder != 999){
    //     SetElbowAngle(tmpMotionProfileElbowHolder);
    //     tmpMotionProfileElbowHolder = 999;
    //   }

    //   if (tmpMotionProfileWristHolder != 999){
    //     SetWristAngle(tmpMotionProfileWristHolder);
    //     tmpMotionProfileWristHolder = 999;
    //   }

    // }

    




    //     // if (GetElbowAngle() < 60 && elbowOutput > 0){
    //     //   elbowOutput *= 0.35;
          
    //     // }
    //     // if (GetElbowAngle() > 135 && elbowOutput < 0){
    //     //   elbowOutput *= 0.75;
          
    //     // }
    //   if (!ElbowPID.atSetpoint() && ((CanMoveElbowDec() && elbowOutput > 0) || (CanMoveElbowInc() && elbowOutput < 0))) 
    //   {
    //     ElbowMotor.set(elbowOutput); // 10% speed for testing;
    //     DogLog.log("CoralElevatorSS/Elbow/ActualElbowMotorOutput", elbowOutput);
    //   }
    //   else{
    //     ElbowMotor.set(0);
    //     DogLog.log("CoralElevatorSS/Elbow/ActualElbowMotorOutput", 0);
    //   }



    //   if (!WristPID.atSetpoint()){
    //     DogLog.log("CoralElevatorSS/Wrist/ActualWristMotorOutput", wristOutput); 
    //     WristMotor.set(wristOutput);
    //   }
    //   else{
    //     WristMotor.set(0);
    //     DogLog.log("CoralElevatorSS/Wrist/ActualWristMotorOutput", 0); 

    //   }
  
    // Datahighway

      

    DH_Out_HasCoral = isCoralPresent();
    DH_Out_ScoreSelection = GetScoreSelection();


  }
  
  //simulation Periodic was here


  private boolean getUpStop() {
    return !ElevatorUpStop.get();
  }

  private boolean getDownStop() {
    return !ElevatorDownStop.get();
  }

  public boolean isCoralPresent() {
    if(Robot.isSimulation()){
      return true;
    }
    return !ElevatorCoralPresence.get();
  }

  private void getElevatorHeightMM() {
    LaserCan.Measurement measurement = LaserCan.getMeasurement();

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      elevatorHeightMM = measurement.distance_mm;
    }
  }

  private void FreezeSetpoints(boolean oElevator, boolean oElbow, boolean oWrist){
      if (!oElbow){
      tmpElbowSetpointHolder = ElbowPID.getSetpoint();
      ElbowPID.setSetpoint(GetElbowAngle());
      }
      if (!oElevator){
        tmpElevatorSetpointHolder = ElevatorPID.getSetpoint();
      ElevatorPID.setSetpoint(getFilteredElevatorHeight());
      }
      if (!oWrist){
        tmpWristSetpointHolder = WristPID.getSetpoint();
        WristPID.setSetpoint(GetWristAngleWorldCoordinates());
      }
      SetpointsFrozen = true;
  }

  private void UnfreezeSetPoints(){
    if (tmpElbowSetpointHolder != 999){
     // SetElbowAngle(tmpElbowSetpointHolder);
    }
    if (tmpElevatorSetpointHolder != 999){
      //SetElevatorPosition(tmpElevatorSetpointHolder);
    }
    if (tmpWristSetpointHolder != 999){
      //SetElevatorPosition(tmpWristSetpointHolder); /// this is very very very wrong!!!!!
    }
    SetpointsFrozen = false;
  }

  //helper functions to facilitate crossing the impassable zone (make the impossible possible)


  private Boolean isElevatorPassingThroughRedZone(){
    return ((getFilteredElevatorHeight() <= RobotLimits.Elevator.elevatorRedZoneLowerHeight && ElevatorPID.getSetpoint() >= RobotLimits.Elevator.elevatorRedZoneLowerHeight ) || (getFilteredElevatorHeight() >= RobotLimits.Elevator.elevatorRedZoneUpperHeight && ElevatorPID.getSetpoint() <= RobotLimits.Elevator.elevatorRedZoneUpperHeight ));
  }

  private Boolean isElevatorInRedZone(){
    double currentheight = getFilteredElevatorHeight();
    return currentheight >= RobotLimits.Elevator.elevatorRedZoneLowerHeight && currentheight <= RobotLimits.Elevator.elevatorRedZoneUpperHeight;
  }
  
  private Boolean isWristPassingThroughVertical(){
    return (GetWristAngleWorldCoordinates() <= 90 && WristPID.getSetpoint() > 90 || GetWristAngleWorldCoordinates() >= 90 && WristPID.getSetpoint() < 90);
  }

  


  

 

  //bunch of helper functions to elevator inputs into meters for sysid

  public static Angle convertDistanceToRotations(Distance distance)
  {
      return Rotations.of(distance.in(Meters) /
                          (PIDs.CoralElevator.Elevator.pulleyRadius * 2 * Math.PI) *
                          PIDs.CoralElevator.Elevator.elevatorReduction);
  }

  public static Distance convertRotationsToDistance(Angle rotations)
  {
    return Meters.of((rotations.in(Rotations) / PIDs.CoralElevator.Elevator.elevatorReduction) *
                      (PIDs.CoralElevator.Elevator.pulleyRadius * 2 * Math.PI));
  }

  public void resetEncoderCount(){
    double encoderCount = (getHeightLaserMeters() / (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius)) * PIDs.CoralElevator.Elevator.elevatorReduction;
      double elevatorHeight = (encoderCount / PIDs.CoralElevator.Elevator.elevatorReduction) *
      (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
    DogLog.log("CoralElevatorSS/Calibration/CalculatedResetEncoder", encoderCount);
    DogLog.log("CoralElevatorSS/Calibration/LaserHeightM", getHeightLaserMeters());
    DogLog.log("CoralElevatorSS/Calibration/CurrentEncoderCount", ElevatorEncoder.getPosition());
    DogLog.log("CoralElevatorSS/Calibration/EncoderHeightM", getHeightMeters());
    DogLog.log("CoralElevatorSS/Calibration/CalculatedEncoderHeightM", elevatorHeight);

    
  }
  public void setBrake(){
    if (ElevatorStageMotor.configAccessor.getIdleMode() != IdleMode.kBrake){
      elevatorConfig.idleMode(IdleMode.kBrake);
      ElevatorStageMotor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
 
    
  }
  public void setCoast(){
    elevatorConfig.idleMode(IdleMode.kCoast);
    ElevatorStageMotor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  
  public double getHeightMeters()
  {
    double elevatorHeight = (ElevatorEncoder.getPosition() / PIDs.CoralElevator.Elevator.elevatorReduction) *
    (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
     DogLog.log("CoralElevatorSS/Elevator/EncoderHeight", elevatorHeight);
    // elevatorHeight = ElevatorEncoder.getPosition() * 0.0104;
    return elevatorHeight;
  }

  public double getHeightLaserMeters(){
    /*
    LaserCan.Measurement measurement= LaserCan.getMeasurement();

    double elevatorHeight = 0;
    try {
      elevatorHeight = measurement.distance_mm / 1000.0 -0.06;
      DogLog.log("CoralElevatorSS/Elevator/Laser Height", elevatorHeight);
        
    } catch (Exception e) {
      // TODO: handle exception
    }
    // return elevatorHeight;
    if (elevatorHeight < 0.3)
      return elevatorHeight;
    else
      return getHeightMeters();

      */
    double elevatorHeight = ElevEnc.get() * Units.inchesToMeters(31.25) / (10432);
    // double elevatorHeight = ((ElevEnc.get()/8192) / PIDs.CoralElevator.Elevator.elevatorReduction) *
    // (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
     DogLog.log("CoralElevatorSS/Elevator/EncoderHeight", elevatorHeight);
     DogLog.log("CoralElevatorSS/Elevator/EncoderDistance", ElevEnc.getDistance());
    // elevatorHeight = ElevatorEncoder.getPosition() * 0.0104;
    return elevatorHeight;

  }

  public double getElevDistance(){
    return ElevEnc.getDistance();
  }

  public double getVelocityMetersPerSecond()
  {
    return ElevEnc.getRate();
    // return ((ElevatorEncoder.getVelocity()/ 60 )/ PIDs.CoralElevator.Elevator.elevatorReduction) *
    //        (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
  }

  public LinearVelocity getLinearVelocity()
  {
    return convertRotationsToDistance(Rotations.of(ElevatorEncoder.getVelocity())).per(Minute);
  }

  public Distance getLinearPosition()
  {
    return convertRotationsToDistance(Rotations.of(ElevatorEncoder.getPosition()));
  }

  //Simulation stuff

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    
    // DogLog.log("Simulation/Elevator",SimElevator);

  }

  /*     if (Robot.isSimulation())
  {
    SimElevatorRoot = SimElevator.getRoot("elevator", 0.1, 0);
    Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
    Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
    Wrist = Elbow.append(new MechanismLigament2d("Wrist",0.2,90));
  }
 */


  private void setupSim()
  {
    SimElevatorRoot = SimElevator.getRoot("elevator", .95, 0.0);
    SimStage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
    SimElbow = SimStage1.append(new MechanismLigament2d("Elbow",0.5,170));
    SimWrist = SimElbow.append(new MechanismLigament2d("Wrist",0.2,90));

    SimStage1.setColor(new Color8Bit(Color.kBlue));
    SimElbow.setColor(new Color8Bit(Color.kGreen));
    SimWrist.setColor(new Color8Bit(Color.kOrange));
  }
  private void applyOutputToSim()
  {
    double ElevatorSpd = ElevatorStageMotor.get();
    double ElbowSpd = -ElbowMotor.get();
    double WristSpd = -WristMotor.get();
    if (DriverStation.isDisabled())
    { 
      ElevatorSpd = 0;
      ElbowSpd = 0;
      WristSpd = 0;
    }
   // if (ElevatorSpd > 0.3 || ElevatorSpd < 0.3)
       SimElevatorHeight = SimElevatorHeight + 0.3*ElevatorSpd;
    if (SimElevatorHeight > 0.9) SimElevatorHeight = 0.9; 
    SimElbowAngle = SimElbowAngle + 3*ElbowSpd;        
    if (SimElbowAngle > 360) SimElbowAngle = SimElbowAngle -360;
    if (SimElbowAngle < -360) SimElbowAngle = SimElbowAngle +360;
    
    SimWristAngle = SimWristAngle + 3*WristSpd;
    if (SimWristAngle > 360) SimWristAngle = SimWristAngle -360;
    SimStage1.setLength(SimElevatorHeight+0.9);
    SimElbow.setAngle(SimElbowAngle+180);
    
    SimWrist.setAngle(SimWristAngle-45);
    SmartDashboard.putData("Elevator",SimElevator);


    if (DH_SimIn_HasCoral)
    {
      double z = SimStage1.getLength() + Math.sin(Units.degreesToRadians(SimElbow.getAngle()))*SimElbow.getLength();
      double x = Math.cos(Units.degreesToRadians(SimElbow.getAngle())*SimElbow.getLength());
      double y = Math.sin(Units.degreesToRadians(SimElbow.getAngle())*SimElbow.getLength());//-Units.inchesToMeters(6);;
      Pose2d RobotPose = DH_In_RobotPose;
      Pose3d CoralPose= new Pose3d(RobotPose.getX()+x,RobotPose.getY()+y,z,new Rotation3d(0,Units.degreesToRadians(90+GetWristAngleWorldCoordinates()),RobotPose.getRotation().getRadians()));
      DogLog.log("Simulation/IntakeCoral",CoralPose);

    }
  }
  


//  SingleJointedArmSim m_elbowSim = new SingleJointedArmSim(ElbowGearbox, 45, 1, 0.30, Units.degreesToRadians(-10) , Units.degreesToRadians(195), true, Units.degreesToRadians(0), null);
//  ElevatorSim m_ElevSim = new ElevatorSim(null, ElbowGearbox, 0, .25, false, 0, null);
//  SingleJointedArmSim m_wristSim = new SingleJointedArmSim(ElbowGearbox, 16, 1, Units.inchesToMeters(12), Units.degreesToRadians(-90) , Units.degreesToRadians(180), true, Units.degreesToRadians(90), null);
  /*******************************

  // SysID nonsense

  // Mutable holder for unit-safe values, persisted to avoid reallocation.
*/
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final MutAngularVelocity m_AngularVelocity = RotationsPerSecond.mutable(0);

  //Elevator
  private final SysIdRoutine ElevatorSysID = new SysIdRoutine
  (new SysIdRoutine.Config(null, Voltage.ofBaseUnits(7,edu.wpi.first.units.Units.Volt), null),
   new SysIdRoutine.Mechanism(
      ElevatorStageMotor::setVoltage,
   // (voltage) -> this.sysidElevatorRunVoltage(voltage.in(Volts)),
    log -> {
      DogLog.log("SysID/Elevator/VoltageApplied", ElevatorStageMotor.getAppliedOutput() * ElevatorStageMotor.getBusVoltage());
      DogLog.log("SysID/Elevator/Position", m_distance.mut_replace(getElevDistance(),Meters).in(Meters));
      DogLog.log("SysID/Elevator/Velocity", m_velocity.mut_replace(getVelocityMetersPerSecond(),MetersPerSecond).in(MetersPerSecond));
    },
    this));

  public void sysidElevatorRunVoltage(double V)
  {
    if (!getUpStop() && !getDownStop())
    ElevatorStageMotor.setVoltage(V);
    else
    ElevatorStageMotor.setVoltage(0);
  }

  public Command sysIDElevatorAll(){
    return (ElevatorSysID.dynamic(Direction.kForward).until(atElevatorMax)
        .andThen(ElevatorSysID.dynamic(Direction.kReverse).until(atElevatorMin))
        .andThen(ElevatorSysID.quasistatic(Direction.kForward).until(atElevatorMax))
        .andThen(ElevatorSysID.quasistatic(Direction.kReverse).until(atElevatorMin))
        .andThen(Commands.print("DONE")));
  }


  //Elbow

  public Command sysIDElbowAll(){
    return (ElbowSysID.dynamic(Direction.kForward).until(atElbowMax)
        .andThen(ElbowSysID.dynamic(Direction.kReverse).until(atElbowMin))
        .andThen(ElbowSysID.quasistatic(Direction.kForward).until(atElbowMax))
        .andThen(ElbowSysID.quasistatic(Direction.kReverse).until(atElbowMin))
        .andThen(Commands.print("DONE")));
  }

    private final SysIdRoutine ElbowSysID = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
      ElbowMotor::setVoltage,
      log -> {
        DogLog.log("SysID/Elbow/VoltageApplied", ElbowMotor.getAppliedOutput() * ElbowMotor.getBusVoltage());
        DogLog.log("SysID/Elbow/Position", m_rotations.mut_replace(ElbowAbsoluteEncoder.getPosition(),Degrees).in(Degrees));
        //check to make sure these units make sense. Normally rev returns velocity in rpm, we've converted it to degrees, so velocity should return as degrees per min
        //for sysid, we want <units> per second for velocity
        DogLog.log("SysID/Elbow/Velocity", m_AngularVelocity.mut_replace(ElbowAbsoluteEncoder.getVelocity(),DegreesPerSecond).in(DegreesPerSecond)); 

      },
      this));

  //Wrist

  public Command sysIDWristAll(){
    return (WristSysID.dynamic(Direction.kForward).until(atWristMax)
        .andThen(WristSysID.dynamic(Direction.kReverse).until(atWristMin))
        .andThen(WristSysID.quasistatic(Direction.kForward).until(atWristMax))
        .andThen(WristSysID.quasistatic(Direction.kReverse).until(atWristMin))
        .andThen(Commands.print("DONE")));
  }

    private final SysIdRoutine WristSysID = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
      WristMotor::setVoltage,
      log -> {
        DogLog.log("SysID/Wrist/VoltageApplied", WristMotor.getAppliedOutput() * WristMotor.getBusVoltage());
        DogLog.log("SysID/Wrist/Position", m_rotations.mut_replace(GetWristAngleWorldCoordinates(),Degrees).in(Degrees));
        DogLog.log("SysID/Wrist/Velocity", m_AngularVelocity.mut_replace(WristAbsoluteEncoder.getVelocity(),DegreesPerSecond).in(DegreesPerSecond)); 

      },
      this));

    /******************************/

    private DoubleSupplier getWristWorldSupplier(){
      DoubleSupplier d = () -> GetWristAngleWorldCoordinates();
      return d;
    }
    private DoubleSupplier getElbowAngleSupplier(){
      DoubleSupplier d = () -> GetElbowAngle();
      return d;
    }
    private DoubleSupplier getElevatorHeightSupplier(){
      DoubleSupplier d = () -> getFilteredElevatorHeight();
      return d;
    }

    private List<Node> CommandQueue = new ArrayList<>();
    private MechState currentState = MechState.StartingConfig;
    private MechState targetState;
    private HashMap<MechState,Node> ConfigList = new HashMap<>();
    private void PopulateConfigList(){ 
        ConfigList.put(MechState.StartingConfig,new Node(StartingConfig.name, StartingConfig.wrist,StartingConfig.elbow,StartingConfig.height));
        ConfigList.put(MechState.AlgaeHigh,new Node(AlgaeL3.name, AlgaeL3.wrist,AlgaeL3.elbow,AlgaeL3.height));
        ConfigList.put(MechState.AlgaeLow,new Node(AlgaeL2.name, AlgaeL2.wrist,AlgaeL2.elbow,AlgaeL2.height));
        ConfigList.put(MechState.CoralStation,new Node(CoralStation.name, CoralStation.wrist,CoralStation.elbow,CoralStation.height));
        ConfigList.put(MechState.Level1,new Node(Level1.name, Level1.wrist,Level1.elbow,Level1.height));
        ConfigList.put(MechState.Level2,new Node(Level2.name, Level2.wrist,Level2.elbow,Level2.height));
        ConfigList.put(MechState.Level3,new Node(Level3.name, Level3.wrist,Level3.elbow,Level3.height));
        ConfigList.put(MechState.Level4,new Node(Level4.name, Level4.wrist,Level4.elbow,Level4.height));
        ConfigList.put(MechState.SafeTransition,new Node(SafeTransition.name, SafeTransition.wrist,SafeTransition.elbow,SafeTransition.height));
        ConfigList.put(MechState.TransportLow,new Node(TransportCoralDown.name, TransportCoralDown.wrist,TransportCoralDown.elbow,TransportCoralDown.height));
        ConfigList.put(MechState.TransportHigh,new Node(TransportCoralUp.name, TransportCoralUp.wrist,TransportCoralUp.elbow,TransportCoralUp.height));
        ConfigList.put(MechState.Holding,new Node(MechState.Holding, getWristWorldSupplier(), getElbowAngleSupplier(), getElevatorHeightSupplier()));

      }

    private void setArmPosition(MechState target){
      if (!CommandQueue.isEmpty()){
        CommandQueue.clear();
      }
      if (!MechAtGoal()){
      currentState = TrajectoryPlanner4920.getClosestNode(ConfigList, new Node(null, GetWristAngleWorldCoordinates(), GetElbowAngle(), getFilteredElevatorHeight()));
      }
      if (currentState.equals(MechState.CoralStation) ^ target.equals(MechState.CoralStation)){
        CommandQueue.add(ConfigList.get(target));
        targetState = MechState.SafeTransition;
      }
      else{
        targetState = target;
      }
      m_controller.setGoal(new State(ConfigList.get(targetState).getElevatorGoal(), 0));
      m_elbowcontroller.setGoal(new State(ConfigList.get(targetState).getElbowGoal(), 0));
      m_wristcontroller.setGoal(new State(ConfigList.get(targetState).getWristGoal(), 0));
        
    }

    private boolean MechAtGoal(){
      return m_controller.atGoal() && m_elbowcontroller.atGoal() && m_wristcontroller.atGoal();
    }

    private void MoveMech(){
      if (!DH_In_RedZone || (DH_In_RedZone && OverrideRedZone)){
        if (MechAtGoal() && currentState != MechState.StartingConfig && currentState != MechState.Holding){
          if (!CommandQueue.isEmpty()){
            currentState = targetState;
            Node n = CommandQueue.remove(0);
            targetState = n.getName();
            m_controller.setGoal(new State(n.getElevatorGoal(), 0));
            m_elbowcontroller.setGoal(new State(n.getElbowGoal(), 0));
            m_wristcontroller.setGoal(new State(n.getWristGoal(), 0));
          }
          else{
            currentState = targetState;
          }
        }
        else{
          if (currentState != targetState){
          m_controller.setGoal(new State(ConfigList.get(targetState).getElevatorGoal(), 0));
          m_elbowcontroller.setGoal(new State(ConfigList.get(targetState).getElbowGoal(), 0));
          m_wristcontroller.setGoal(new State(ConfigList.get(targetState).getWristGoal(), 0));
          }
        }
      }
    else{
      m_controller.setGoal(new State(ConfigList.get(MechState.Holding).getElevatorGoal(), 0));
      m_elbowcontroller.setGoal(new State(ConfigList.get(MechState.Holding).getElbowGoal(), 0));
      m_wristcontroller.setGoal(new State(ConfigList.get(MechState.Holding).getWristGoal(), 0));
      currentState = MechState.Holding;
      
    }
  }
}

