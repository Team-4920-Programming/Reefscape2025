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

import java.util.concurrent.ThreadPoolExecutor.AbortPolicy;

import javax.lang.model.util.ElementScanner14;

import org.dyn4j.geometry.decompose.EarClipping;
import org.opencv.core.Mat;

import com.fasterxml.jackson.databind.BeanProperty;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.Constants.RobotPositions.CoralStation;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Constants.RobotMotionLimits;
import frc.robot.Robot;

import au.grapplerobotics.LaserCan;
import dev.doglog.DogLog;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;

public class CoralElevatorSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final BoltLog BoltLogger = new BoltLog();
  private Mechanism2d SimElevator = new Mechanism2d(1, 1);
  private MechanismRoot2d SimElevatorRoot;
  private MechanismLigament2d Stage1;
  private MechanismLigament2d Elbow;
  private MechanismLigament2d Wrist;
  
  // DataHighway
  public boolean DH_Out_HasCoral = false;
  public double DH_In_DistanceFromReef = 0;
  public boolean DH_In_RedZone = true;
  public boolean DH_In_YellowZone = false;
  public boolean DH_In_CoralZone = false;
  public Pose2d DH_In_RobotPose = new Pose2d();
  public boolean OverrideRedZone = false; //set from the algae commands
  public int DH_Out_ScoreSelection = 1;

  /** Physical Robot Init START**/
  
  // Motors

  SparkMax ElevatorStageMotor = new SparkMax(CanIDs.CoralElevator.ElevatorStage, MotorType.kBrushless);
  SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  SparkMax ElbowMotor = new SparkMax(CanIDs.CoralElevator.Elbow, MotorType.kBrushless);
  SparkMaxConfig elbowConfig = new SparkMaxConfig();
  SparkMax WristMotor = new SparkMax(CanIDs.CoralElevator.Wrist, MotorType.kBrushless);
  SparkMax CoralIntakeMotor = new SparkMax(CanIDs.CoralElevator.CoralIntake, MotorType.kBrushless);
  SparkMaxConfig CoralIntakeConfig = new SparkMaxConfig();
 // SparkMax CoralFlapLeft = new SparkMax(CanIDs.CoralElevator.CoralFlapLeft, MotorType.kBrushless);

 // SparkMaxConfig CoralFlapLeftConfig = new SparkMaxConfig();
  

  // PIDs

  ElevatorFeedforward ElevFF = new ElevatorFeedforward(Elevator.ks, Elevator.kg, Elevator.kv,Elevator.ka);
  ArmFeedforward ElbowFF = new ArmFeedforward(PIDs.CoralElevator.Elbow.ks, PIDs.CoralElevator.Elbow.kg, PIDs.CoralElevator.Elbow.kv);
  // ProfiledPIDController ElevatorPID = new ProfiledPIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd, new Constraints(PIDs.CoralElevator.Elevator.maxVelocity,PIDs.CoralElevator.Elevator.maxAcceleration));
  PIDController ElevatorPID = new PIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd);
  PIDController ElbowPID = new PIDController(PIDs.CoralElevator.Elbow.kp, PIDs.CoralElevator.Elbow.ki, PIDs.CoralElevator.Elbow.kd);

  PIDController WristPID = new PIDController(PIDs.CoralElevator.Wrist.kp, PIDs.CoralElevator.Wrist.ki, PIDs.CoralElevator.Wrist.kd);
  
 // PIDController LeftFlapPID = new PIDController(PIDs.CoralElevator.LeftFlap.kp,PIDs.CoralElevator.LeftFlap.ki,PIDs.CoralElevator.LeftFlap.kd );
  // Limit Switch

  DigitalInput ElevatorUpStop = new DigitalInput(DIO.CoralElevator.UpStop);
  DigitalInput ElevatorDownStop = new DigitalInput(DIO.CoralElevator.DownStop);
  DigitalInput ElevatorCoralPresence = new DigitalInput(DIO.CoralElevator.CoralPresence);
  Ultrasonic DistanceSensor = new Ultrasonic(5, 6);
  //DigitalInput UltrasonicTrigger = new DigitalInput(DIO.CoralElevator.UltrasonicTrigger);

  // Encoders

  RelativeEncoder ElevatorEncoder = ElevatorStageMotor.getEncoder();
  AbsoluteEncoder ElbowAbsoluteEncoder = ElbowMotor.getAbsoluteEncoder();
  RelativeEncoder ElbowRelativeEncoder = ElbowMotor.getEncoder();
  AbsoluteEncoder WristAbsoluteEncoder = WristMotor.getAbsoluteEncoder();
  RelativeEncoder WristRelative = WristMotor.getEncoder();
 // AbsoluteEncoder CoralFlapLeftEncoder = CoralFlapLeft.getAbsoluteEncoder();
 
  // LaserCan
  private LaserCan LaserCan = new LaserCan(CanIDs.Sensor.LaserCAN);
  int elevatorHeightMM = 15;

  //Elevator Low Pass Filter
  LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  double filteredelevatorHeight = 0.15;

  //MotorOutputs

  double elevatorOutput;
  double elbowOutput;
  double wristOutput;
  boolean SetpointsFrozen = false;
  /** Physical Robot Init END**/

  double tmpElbowSetpointHolder = 999;

  /** SIM Robot Init START */
  DCMotor ElevatorStageGearbox = DCMotor.getNEO(1);
  DCMotor ElbowGearbox = DCMotor.getNEO(1);
  DCMotor WristGearbox = DCMotor.getNEO(1);
  DCMotor CoralIntakeGearbox = DCMotor.getNEO(1);

  // simulated motors were here

  /** SIM Robot Init END */

  // states
  boolean SetFlapIntake = true;
  boolean SetFlapClimb = false;


  private int ScoreSelection = 4; //default score level is 4

  public final Trigger atElevatorMin = new Trigger(() -> !CanMoveElevatorDown());
  public final Trigger atElevatorMax = new Trigger(() -> !CanMoveElevatorUp());
  public final Trigger atElbowMin = new Trigger(() -> !CanMoveElbowDec());
  public final Trigger atElbowMax = new Trigger(() -> !CanMoveElbowInc());
  public final Trigger atWristMin = new Trigger(() -> !CanMoveWristDec());
  public final Trigger atWristMax = new Trigger(() -> !CanMoveWristInc());
  
  public CoralElevatorSubsystem() {


    ElevatorPID.setTolerance(0.01);
    //ElevatorPID.setSetpoint(Constants.RobotLimits.Elevator.offset);
    // ElevatorPID.setGoal(new State(Constants.RobotLimits.Elevator.offset, 0));
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.inverted(false);
    ElevatorStageMotor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    ElbowPID.setTolerance(3);
    //ElbowPID.setSetpoint(5);
   // ElbowPID.enableContinuousInput(0, 360);

    elbowConfig.idleMode(IdleMode.kBrake);
    elbowConfig.inverted(true);
    elbowConfig.absoluteEncoder.positionConversionFactor(360);
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

    //elbowConfig.encoder.positionConversionFactor(2.37);
    elbowConfig.inverted(true);
    ElbowMotor.configure(elbowConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    WristPID.setTolerance(3);
    // if robot.issimulation was here
    ElbowPID.setSetpoint(GetElbowAngle());
    ElevatorPID.setSetpoint(Math.max(getFilteredElevatorHeight(),0.16));
    CoralIntakeConfig.inverted(true);
    CoralIntakeMotor.configure(CoralIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    WristPID.setSetpoint(GetWristAngleWorldCoordinates());
    // WristPID.enableContinuousInput(-180, 180);
    // WristPID
    

    ScoreSelection = 4; //default to level 4

  }
  //Flap Functions
  
  public void SetLeftFlap(double Angle)
  {
   // LeftFlapPID.setSetpoint(Angle);
  }


  public double GetLeftFlap()
  {
    return 0;//CoralFlapLeftEncoder.getPosition();
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
    if (!SetpointsFrozen){
      ElevatorPID.setSetpoint(height);
    }
    // if (height > getFilteredElevatorHeight() && !SetpointsFrozen){ //&& CanMoveElevatorUp()&& !SetpointsFrozen){
    //   // ElevatorPID.setGoal(new State(height, 0));
    //   ElevatorPID.setSetpoint(height); //(new State(height, 0));
    // }
    // else if (height < getFilteredElevatorHeight() && CanMoveElevatorDown()&& !SetpointsFrozen){
    //   ElevatorPID.setSetpoint(height); //(new State(height, 0));
    // }
  }

  public boolean IsElevatorAtSetpoint(){
    return ElevatorPID.atSetpoint();
  }
  public boolean IsWristAtSetpoint(){
    return WristPID.atSetpoint();
  }
  public boolean IsElbowAtSetpoint(){
    return ElbowPID.atSetpoint();
  }
  
  private Boolean ElevatorClearToMoveCheck(){
    Boolean elbowClear = false;
    boolean wristClear = false;
    if (GetElbowAngle() > RobotMotionLimits.Elbow.minAngle && GetElbowAngle() < RobotMotionLimits.Elbow.maxAngle )
    {
      elbowClear = true;
    }

    //wrist > 90 0.36 cant go up
    //0.55 cant go down
    if (GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle  && GetWristAngleWorldCoordinates() < 180 )
    {
      wristClear = true;
    }

    return elbowClear && wristClear;
  }
  
  private Boolean CanMoveElevatorUp(){
    return (!getUpStop() && ElevatorClearToMoveCheck() && !isWristPassingThroughVertical());
  }

  private Boolean CanMoveElevatorDown(){
    return (!getDownStop() && ElevatorClearToMoveCheck() && !isWristPassingThroughVertical());
  }

  /* Elbow */

  public void SetElbowAngle(double angle)
  {
    if (!SetpointsFrozen){
      ElbowPID.setSetpoint(angle);
    }
    // if (angle >= GetElbowAngle()&& !SetpointsFrozen) {
    //   ElbowPID.setSetpoint(angle);
    // }
    // if (angle <= GetElbowAngle()&& !SetpointsFrozen){
    //   ElbowPID.setSetpoint(angle);
    // }
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
    return ElbowAbsoluteEncoder.getPosition()-25;
  }

  /* Wrist */

  public void SetWristAngle(double angle)
  {
    ////System.out.println("Angle = " + angle);
    ////System.out.println("GetWristAngleWorldCoordinates = " + GetWristAngleWorldCoordinates());
    ////System.out.println("CanMoveWristInc = " + CanMoveWristInc());
    ////System.out.println("CanMoveWristDec = " + CanMoveWristDec());
    ////System.out.println("GetWristAngle =" +  GetWristAngle());
    // if (angle > GetWristAngleWorldCoordinates() && CanMoveWristInc() && !SetpointsFrozen){
    //   WristPID.setSetpoint(angle);
    // }
    // if (angle < GetWristAngleWorldCoordinates() && CanMoveWristDec() && !SetpointsFrozen){
    //   WristPID.setSetpoint(angle);
    // }
    if (!SetpointsFrozen){
      WristPID.setSetpoint(angle);
    }
  }

  private Boolean CanMoveWristInc(){
    return GetWristAngleWorldCoordinates() < RobotMotionLimits.Wrist.maxAngle;
  }
  
  private Boolean CanMoveWristDec(){
    return GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle;
  }

  public double GetWristAngle(){
    return WristAbsoluteEncoder.getPosition();
  }

  public double GetWristAngleWorldCoordinates(){
    double wAng = WristAbsoluteEncoder.getPosition();
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
    SmartDashboard.putNumber("IntakeSpeedCmd", speed);
  }

  public boolean isIntakeRunning() {
    return CoralIntakeMotor.get() > 0;
  }

  public void MatchSetup(){
      if(!DH_In_RedZone){
      ElevatorPID.setSetpoint(CoralStation.height);
      }
      else{
      ElevatorPID.setSetpoint(getHeightLaserMeters());
      }
    }

  public void setArmPosition(double height, double elbow, double wrist){
  
    double goalheight = height ;
    double goalwrist = wrist;
    double goalelbow = elbow;

    /*****
       //safety Checks
    boolean ElevatorDirectionUp = false;
    boolean ElevatorDirectionDown = false;
    boolean ElbowUp = false;
    boolean ElbowDown = false;
    boolean WristUp = false;
    boolean WristDown = false;

    if (getFilteredElevatorHeight() < goalheight )
    {
      ElevatorDirectionUp = true;
      ElevatorDirectionDown = false;

    }
    if (getFilteredElevatorHeight() > goalheight )
    {
      ElevatorDirectionDown = true;
      ElevatorDirectionUp = false;
    }

    if (GetElbowAngle() < goalelbow )
    {
      ElbowUp = true;
      ElbowDown = false;

    }
    if (GetElbowAngle() > goalelbow )
    {
      ElbowDown = true;
      ElbowUp = false;
    }
    if (GetWristAngleWorldCoordinates() < goalwrist )
    {
      WristUp = true;
      WristDown = false;

    }
    if (GetWristAngleWorldCoordinates() > goalwrist )
    {
      WristUp = false;
      WristDown = true;

    }

    //Elevator Height Move


    // if (ElevatorDirectionUp && CanMoveElevatorUp())
    // {
        SetElevatorPosition(goalheight);
    // }
    // if (ElevatorDirectionDown && CanMoveElevatorDown())
    // {
    //     SetElevatorPosition(goalheight);
    // }

    //Elbow Move
    // Boolean ElevatorClearForElbowUp = false;
    // Boolean ElevatorClearForElbowDown = false;
    // if (getFilteredElevatorHeight() > RobotMotionLimits.Elevator.minHeight && getFilteredElevatorHeight() < RobotMotionLimits.Elevator.maxHeight)
    // {
    //     ElevatorClearForElbowUp = true;
    //     ElevatorClearForElbowDown = true;
    // }
    // Boolean WristClearforElbowUp = false;
    // Boolean WristClearforElbowDown = false;


    // if (GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle + 10 && GetWristAngleWorldCoordinates() < RobotMotionLimits.Wrist.maxAngle - 5)
    // {
     
    //     WristClearforElbowUp = true;
      
    // }

    // if (GetWristAngleWorldCoordinates() > RobotMotionLimits.Wrist.minAngle + 10)
    // {
     
    //     WristClearforElbowDown = true;
      
    // }

    

    // if (ElbowUp && ElevatorClearForElbowUp && WristClearforElbowUp)
      if (ElbowUp && CanMoveElbowInc()){
      SetElbowAngle(goalelbow);
      }
    // if (ElbowDown && ElevatorClearForElbowDown && WristClearforElbowDown)
      if (ElbowDown && CanMoveElbowDec()){
      SetElbowAngle(goalelbow);
      }

    Boolean ElbowClearForWristUp = false;
    Boolean ElbowClearForWristDown = false;
    // if (GetElbowAngle() >0 && GetElbowAngle() <190)
    if (GetElbowAngle() < 15 && GetWristAngleWorldCoordinates() < -60)
    {
     ElbowClearForWristUp = true;
      ElbowClearForWristDown = false;
    }
    else{
      ElbowClearForWristUp = true;
      ElbowClearForWristDown = true;
    }
    if (WristUp && ElbowClearForWristUp)
    {
      SetWristAngle(goalwrist);
    }
    if (WristDown && ElbowClearForWristDown)
    {
      SetWristAngle(goalwrist);
    }
      *****/

      SetElevatorPosition(goalheight);
      SetElbowAngle(goalelbow);
      SetWristAngle(goalwrist);
  }

  public double getFilteredElevatorHeight(){
    return filteredelevatorHeight;
  }

  public double getUltrasonic(){
      return DistanceSensor.getRangeInches();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // getElevatorHeightMM();
    // readSensorValues();
    // elevatorOutput = ElevatorPID.calculate(elevatorHeightMM) + ElevFF.calculate(PIDs.CoralElevator.Elevator.maxVelocity)/RobotController.getBatteryVoltage();
    
    DistanceSensor.setAutomaticMode(true);
    DistanceSensor.setEnabled(true);
    if (DistanceSensor.isRangeValid())
    {
      SmartDashboard.putNumber("ultasonic", DistanceSensor.getRangeInches());
    }
    filteredelevatorHeight = elevatorFilter.calculate(getHeightLaserMeters());
    if (DH_In_RedZone && !OverrideRedZone  && !SetpointsFrozen){
      //Freeze PIDs at current positon
      ElbowPID.setSetpoint(GetElbowAngle());
      ElevatorPID.setSetpoint(getFilteredElevatorHeight());
      WristPID.setSetpoint(GetWristAngleWorldCoordinates());

      SetpointsFrozen = true;
    }
    if (!DH_In_RedZone){
      SetpointsFrozen = false;
    }
    if (DH_In_RedZone && OverrideRedZone ){
      SetpointsFrozen = false;
    }

    if ((isElevatorPassingThroughRedZone() || isElevatorInRedZone()) && (ElbowPID.getSetpoint() < 0 || WristPID.getSetpoint() >= 90)){
        if (tmpElbowSetpointHolder == 999){
          tmpElbowSetpointHolder = ElbowPID.getSetpoint();
          ElbowPID.setSetpoint(35);
        }
      }
      else if (tmpElbowSetpointHolder != 999){
        ElbowPID.setSetpoint(tmpElbowSetpointHolder);
        tmpElbowSetpointHolder = 999 ;
      }

    filteredelevatorHeight = elevatorFilter.calculate(getHeightLaserMeters());
    double elevatorPIDValue = ElevatorPID.calculate(filteredelevatorHeight);
    double linearVelocity = getVelocityMetersPerSecond();
    double elevatorFFValue = 0.6;//ElevFF.calculate(linearVelocity);
    if (DriverStation.isEnabled()){
      //elevatorOutput = MathUtil.clamp(elevatorPIDValue + elevatorFFValue,-7,7);
      elevatorOutput = MathUtil.clamp(elevatorPIDValue,-1,1);
      elbowOutput = ElbowPID.calculate(GetElbowAngle());
      elbowOutput = -elbowOutput;
      elbowOutput = MathUtil.clamp(elbowOutput, -1.0, 1.0);//was -.5 and .5
      wristOutput = WristPID.calculate(GetWristAngleWorldCoordinates());
    }
    else
    {
        // ElevatorPID.setSetpoint(getFilteredElevatorHeight());
    }
    //double leftFlapOutput = LeftFlapPID.calculate(GetLeftFlap());
    
    
    SmartDashboard.putNumber("Elevator PID Output", elevatorPIDValue);
    SmartDashboard.putNumber("Elevator Output", elevatorOutput);
    SmartDashboard.putNumber("Elevator FF Output", elevatorFFValue);
    SmartDashboard.putNumber("Elevator Setpount", ElevatorPID.getSetpoint());
    SmartDashboard.putNumber("Elevator Error", ElevatorPID.getError());
    
    SmartDashboard.putNumber("ElevatorVelocity", linearVelocity);
    SmartDashboard.putNumber("Elevator Height", getFilteredElevatorHeight());
    SmartDashboard.putBoolean("CanMoveElevatorUp", CanMoveElevatorUp());
    SmartDashboard.putBoolean("CanMoveElevatorDown", CanMoveElevatorDown());
    SmartDashboard.putBoolean("ElevatorAtGoal", ElevatorPID.atSetpoint());
    SmartDashboard.putNumber("Elevator Current", ElevatorStageMotor.getOutputCurrent());
  
    //Elbow Data
    SmartDashboard.putNumber("ElbowAngle", GetElbowAngle());
    
    SmartDashboard.putNumber("Elbow Setpoint", ElbowPID.getSetpoint());
    SmartDashboard.putBoolean("CanIncElbow", CanMoveElbowInc());
    SmartDashboard.putBoolean("CanDecElbow", CanMoveElbowDec());

    SmartDashboard.putNumber("Wrist Output", wristOutput);
    SmartDashboard.putNumber("Wrist Setpoint", WristPID.getSetpoint());
    SmartDashboard.putNumber("WristAbsolute", WristAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("WristWorld", GetWristAngleWorldCoordinates());
    SmartDashboard.putBoolean("CanIncWrist", CanMoveWristInc());
    SmartDashboard.putBoolean("CanDecWrist", CanMoveWristDec());

    SmartDashboard.putNumber("LeftFlap", GetLeftFlap());

    SmartDashboard.putBoolean("ElevatorClearToMoveCheck",ElevatorClearToMoveCheck());
    SmartDashboard.putBoolean("isWristPassingThroughVertical",!isWristPassingThroughVertical());
    

    SmartDashboard.putBoolean("HasCoral", isCoralPresent());
    SmartDashboard.putNumber("IntakeSpeed", CoralIntakeMotor.get());
    double  EmotorSpd =0;
   
      if (!ElevatorPID.atSetpoint() && ((CanMoveElevatorUp() && elevatorOutput > 0) || (CanMoveElevatorDown() && elevatorOutput < 0)))
      { 
        double EleveFF = 0.025;
        if (getFilteredElevatorHeight() > 0.30 && getFilteredElevatorHeight() <0.5)
          EleveFF = 0.01;

        EmotorSpd = elevatorOutput + EleveFF;
        if (EmotorSpd < 0.05 && EmotorSpd>0)
          EmotorSpd = 0.05;
        if (EmotorSpd > -0.05 && EmotorSpd<0)
          EmotorSpd = -0.05;  
        ElevatorStageMotor.set(EmotorSpd); 
      
      }
      else{ 
        ElevatorStageMotor.set(0.0);
        EmotorSpd = 0;
      }
      SmartDashboard.putNumber("Elevator Output", EmotorSpd);



      SmartDashboard.putNumber("Elbow Output", elbowOutput);
      if (ElbowPID.getSetpoint() > GetElbowAngle()){
        // elbowOutput = MathUtil.clamp(elbowOutput,-.1,.0);
        
      }
      if (ElbowPID.getSetpoint() < GetElbowAngle()){
        // elbowOutput = MathUtil.clamp(elbowOutput, 0,.75);
        
      }
      if (!ElbowPID.atSetpoint() && ((CanMoveElbowDec() && elbowOutput > 0) || (CanMoveElbowInc() && elbowOutput < 0))) 
      {
        SmartDashboard.putString("Elbow Allowed to move", "true") ; 
        ElbowMotor.set(elbowOutput);
      }
        else{
          SmartDashboard.putString("Elbow Allowed to move", "false") ;
          ElbowMotor.set(0);
        }
        
      //wristOutput = - wristOutput;
      if (WristPID.getSetpoint() > GetWristAngleWorldCoordinates()){
        wristOutput = MathUtil.clamp(wristOutput,0,0.75); //0.5
        
      }
      if (WristPID.getSetpoint() < GetWristAngleWorldCoordinates()){
        wristOutput = MathUtil.clamp(wristOutput, -0.75,0); //0.5
        
      }
      if (WristPID.getSetpoint() < -90 && GetWristAngleWorldCoordinates() >0){
        wristOutput = 0.5;//MathUtil.clamp(wristOutput, -.05,0);
        
      }
      if (GetWristAngleWorldCoordinates() < -180 && isCoralPresent() && !WristPID.atSetpoint())
      {
        wristOutput = MathUtil.clamp(wristOutput, -0.5,0);
      }

      if (!WristPID.atSetpoint())  
        WristMotor.set(wristOutput);
      else 
        WristMotor.set(0);
  
    // Datahighway
    SmartDashboard.putNumber("LeftFlapPos", GetLeftFlap());
    //leftFlapOutput = -MathUtil.clamp(leftFlapOutput, -.1, .1);
  
    //SmartDashboard.putNumber("LeftFlapOut", leftFlapOutput);
   // SmartDashboard.putNumber("LeftSetpoint", LeftFlapPID.getSetpoint());
    //CoralFlapLeft.set(-leftFlapOutput);
      //CoralFlapRight.set(-rightFlapOutput);
      

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

  //helper functions to facilitate crossing the impassable zone (make the impossible possible)


  private Boolean isElevatorPassingThroughRedZone(){
    return ((getFilteredElevatorHeight() <= RobotLimits.Elevator.elevatorRedZoneLowerHeight && ElevatorPID.getSetpoint() >= RobotLimits.Elevator.elevatorRedZoneUpperHeight ) || (getFilteredElevatorHeight() >= RobotLimits.Elevator.elevatorRedZoneUpperHeight && ElevatorPID.getSetpoint() <= RobotLimits.Elevator.elevatorRedZoneLowerHeight ));
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

  
  public double getHeightMeters()
  {
    double elevatorHeight = (ElevatorEncoder.getPosition() / PIDs.CoralElevator.Elevator.elevatorReduction) *
    (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
    SmartDashboard.putNumber("Elevator Height", elevatorHeight);
    return elevatorHeight;
  }

  public double getHeightLaserMeters(){
    
    LaserCan.Measurement measurement= LaserCan.getMeasurement();

    double elevatorHeight = 0;
    try {
      elevatorHeight = measurement.distance_mm / 1000.0;
      SmartDashboard.putNumber("Elevator Height", elevatorHeight);
        
    } catch (Exception e) {
      // TODO: handle exception
    }
    return elevatorHeight;
  }

  public double getVelocityMetersPerSecond()
  {
    return ((ElevatorEncoder.getVelocity()/ 60 )/ PIDs.CoralElevator.Elevator.elevatorReduction) *
           (2 * Math.PI * PIDs.CoralElevator.Elevator.pulleyRadius);
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
    if (Robot.isSimulation())
    SmartDashboard.putData  ("Simulation/Elevator",SimElevator);

  }

  /*     if (Robot.isSimulation())
  {
    SimElevatorRoot = SimElevator.getRoot("elevator", 0.1, 0);
    Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
    Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
    Wrist = Elbow.append(new MechanismLigament2d("Wrist",0.2,90));
  }
 */

 SparkMaxSim ElevatorUpDownMotorSim = new SparkMaxSim(ElevatorStageMotor, ElevatorStageGearbox);
 SparkMaxSim ElbowMotorSim = new SparkMaxSim(ElbowMotor, ElbowGearbox);
 SparkMaxSim WristMotorSim = new SparkMaxSim(WristMotor, WristGearbox);
 SparkMaxSim CoralIntakeSim = new SparkMaxSim(CoralIntakeMotor, CoralIntakeGearbox);

 // SingleJointedArmSim m_armSim = new SingleJointedArmSim(ElbowGearbox, 1, 0.1, .1, 0, .5*3.145, false, 0, null) ;
 // ElevatorSim m_ElevSim = new ElevatorSim(null, ElbowGearbox, 0, .25, false, 0, null);



  /*******************************

  // SysID nonsense

  // Mutable holder for unit-safe values, persisted to avoid reallocation.

  private final MutDistance m_distance = Meters.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final MutAngularVelocity m_AngularVelocity = RotationsPerSecond.mutable(0);

  //Elevator
  private final SysIdRoutine ElevatorSysID = new SysIdRoutine
  (new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism(
      ElevatorStageMotor::setVoltage,
   // (voltage) -> this.sysidElevatorRunVoltage(voltage.in(Volts)),
    log -> {
      DogLog.log("SysID/Elevator/VoltageApplied", ElevatorStageMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
      DogLog.log("SysID/Elevator/Position", m_distance.mut_replace(getHeightMeters(),Meters).in(Meters));
      DogLog.log("SysID/Elevator/Velocity", m_velocity.mut_replace(getVelocityMetersPerSecond(),
      MetersPerSecond).in(MetersPerSecond));
    },
    this));

  public void sysidElevatorRunVoltage(double V)
  {
    if (CanMoveElevatorUp() && V > 0)
    ElevatorStageMotor.set(V / RobotController.getBatteryVoltage());
    else if (CanMoveElevatorDown() && V < 0)
    ElevatorStageMotor.set(V / RobotController.getBatteryVoltage());
    else
    ElevatorStageMotor.set(0);
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
        DogLog.log("SysID/Elbow/VoltageApplied", ElbowMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        DogLog.log("SysID/Elbow/Position", m_rotations.mut_replace(ElbowAbsoluteEncoder.getPosition(),Degrees).in(Degrees));
        //check to make sure these units make sense. Normally rev returns velocity in rpm, we've converted it to degrees, so velocity should return as degrees per min
        //for sysid, we want <units> per second for velocity
        DogLog.log("SysID/Elbow/Velocity", m_AngularVelocity.mut_replace(ElbowAbsoluteEncoder.getVelocity(),Degrees.per(Minute)).in(DegreesPerSecond)); 

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
        DogLog.log("SysID/Wrist/VoltageApplied", WristMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        DogLog.log("SysID/Wrist/Position", m_rotations.mut_replace(WristAbsoluteEncoder.getPosition(),Degrees).in(Degrees));
        DogLog.log("SysID/Wrist/Velocity", m_AngularVelocity.mut_replace(WristAbsoluteEncoder.getVelocity(),Degrees.per(Minute)).in(DegreesPerSecond)); 

      },
      this));

    ******************************/
}

