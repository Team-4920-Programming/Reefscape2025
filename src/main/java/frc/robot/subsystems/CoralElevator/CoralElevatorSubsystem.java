// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;

import static edu.wpi.first.units.Units.Volts;

import org.opencv.core.Mat;

import com.fasterxml.jackson.databind.BeanProperty;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.BoltLog;
import frc.robot.Constants.CanIDs.CoralElevator;
import frc.robot.Constants.PIDs.CoralElevator.Elevator;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Robot;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

public class CoralElevatorSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final BoltLog BoltLogger = new BoltLog();
  private Mechanism2d SimElevator = new Mechanism2d(1, 1);
  private MechanismRoot2d SimElevatorRoot;
  private MechanismLigament2d Stage1;
  private MechanismLigament2d Elbow;
  private MechanismLigament2d Wrist;
  

  /** Physical Robot Init START**/
  
  // Motors

  SparkMax ElevatorStageMotor = new SparkMax(CoralElevator.ElevatorStage, MotorType.kBrushless);
  SparkMax ElbowMotor = new SparkMax(CoralElevator.Elbow, MotorType.kBrushless);
  SparkMax WristMotor = new SparkMax(CoralElevator.Wrist, MotorType.kBrushless);
  SparkMax CoralIntakeMotor = new SparkMax(CoralElevator.CoralIntake, MotorType.kBrushless);


  // PIDs

  ElevatorFeedforward ElevFF = new ElevatorFeedforward(Elevator.ks, Elevator.kg, Elevator.kv);
  ArmFeedforward ElbowFF = new ArmFeedforward(PIDs.CoralElevator.Elbow.ks, PIDs.CoralElevator.Elbow.kg, PIDs.CoralElevator.Elbow.kv);
  PIDController ElevatorPID = new PIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd);
  PIDController ElbowPID = new PIDController(PIDs.CoralElevator.Elbow.kp, PIDs.CoralElevator.Elbow.ki, PIDs.CoralElevator.Elbow.kd);
  PIDController WristPID = new PIDController(PIDs.CoralElevator.Wrist.kp, PIDs.CoralElevator.Wrist.ki, PIDs.CoralElevator.Wrist.kd);

  // Limit Switch

  DigitalInput ElevatorUpStop = new DigitalInput(DIO.CoralElevator.UpStop);
  DigitalInput ElevatorDownStop = new DigitalInput(DIO.CoralElevator.DownStop);
  DigitalInput ElevatorCoralPresence = new DigitalInput(DIO.CoralElevator.CoralPresence);

  boolean UpStop;
  boolean DownStop;
  boolean CoralPresent;

  // Encoders

  RelativeEncoder ElevatorEncoder = ElevatorStageMotor.getEncoder();
  AbsoluteEncoder ElbowAbsoluteEncoder = ElbowMotor.getAbsoluteEncoder();
  RelativeEncoder ElbowRelativeEncoder = ElbowMotor.getEncoder();
  AbsoluteEncoder WristAbsoluteEncoder = WristMotor.getAbsoluteEncoder();
  RelativeEncoder WristRelative = WristMotor.getEncoder();

  double ElbowAngle;
  double WristAngle;

  // LaserCan
  private LaserCan LaserCan = new LaserCan(CanIDs.Sensor.LaserCAN);
  int elevatorHeightMM = 0;


  //MotorOutputs

  double elevatorOutput;
  double elbowOutput;
  double wristOutput;

  /** Physical Robot Init END**/

  /** SIM Robot Init START */
  DCMotor ElevatorStageGearbox = DCMotor.getNEO(1);
  DCMotor ElbowGearbox = DCMotor.getNEO(1);
  DCMotor WristGearbox = DCMotor.getNEO(1);
  DCMotor CoralIntakeGearbox = DCMotor.getNEO(1);

  SparkMaxSim ElevatorUpDownMotorSim = new SparkMaxSim(ElevatorStageMotor, ElevatorStageGearbox);
  SparkMaxSim ElbowMotorSim = new SparkMaxSim(ElbowMotor, ElbowGearbox);
  SparkMaxSim WristMotorSim = new SparkMaxSim(WristMotor, WristGearbox);
  SparkMaxSim CoralIntakeSim = new SparkMaxSim(CoralIntakeMotor, CoralIntakeGearbox);

  // SingleJointedArmSim m_armSim = new SingleJointedArmSim(ElbowGearbox, 1, 0.1, .1, 0, .5*3.145, false, 0, null) ;
  // ElevatorSim m_ElevSim = new ElevatorSim(null, ElbowGearbox, 0, .25, false, 0, null);


  /** SIM Robot Init END */
  



  public CoralElevatorSubsystem() {
    if (Robot.isSimulation())
   {
      SimElevatorRoot = SimElevator.getRoot("elevator", 0.5, 0);
      Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
      Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
      Wrist = Elbow.append(new MechanismLigament2d("Wriat",0.2,90));
    }
  }
  private void SetElevatorPosition(double height)
{
  if (height > ElevatorPID.getSetpoint() && CanMoveElevatorUp()){
    ElevatorPID.setSetpoint(height);
  }
  if (height < ElevatorPID.getSetpoint() && CanMoveElevatorDown()){
    ElevatorPID.setSetpoint(height);
  }
}

  private void SetElbowPosition(double angle)
{
  if (angle > ElbowPID.getSetpoint() && CanMoveElbowInc()){
    ElbowPID.setSetpoint(angle);
  }
  if (angle < ElbowPID.getSetpoint() && CanMoveElbowDec()){
    ElbowPID.setSetpoint(angle);
  }
}
  private void SetWristPosition(double angle)
{
  if (angle > WristPID.getSetpoint() && CanMoveWristInc()){
    WristPID.setSetpoint(angle);
  }
  if (angle < WristPID.getSetpoint() && CanMoveWristDec()){
    WristPID.setSetpoint(angle);
  }
}
  

    

private Boolean CanMoveElevatorUp(){
  return !UpStop;
}
private Boolean CanMoveElevatorDown(){
  return !DownStop;
}
private Boolean CanMoveElbowInc(){
  return ElbowAngle < RobotLimits.Elbow.maxAngle;
}
private Boolean CanMoveElbowDec(){
  return ElbowAngle > RobotLimits.Elbow.minAngle;
}
private Boolean CanMoveWristInc(){
  return WristAngle < RobotLimits.Wrist.maxAngle;
}
private Boolean CanMoveWristDec(){
  return WristAngle > RobotLimits.Wrist.minAngle;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readSensorValues();
   
    elevatorOutput = ElevatorPID.calculate(elevatorHeightMM) + ElevFF.calculate(PIDs.CoralElevator.Elevator.maxVelocity);
    elbowOutput = ElbowPID.calculate(ElbowAngle) + Math.toDegrees(ElbowFF.calculate (Math.toRadians(ElbowAbsoluteEncoder.getPosition()), PIDs.CoralElevator.Elbow.maxVelocity));
    wristOutput = WristPID.calculate(WristAngle);

    // ElevatorStageMotor.set(elevatorOutput);
    // ElbowMotor.set(elbowOutput);
    // WristMotor.set(wristOutput);

  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation())
    SmartDashboard.putData  ("Simulation/Elevator",SimElevator);

  }
public  SysIdRoutine ElevatorSysIDRun(Config config)
{
  return new SysIdRoutine(
  config,
   new SysIdRoutine.Mechanism(
    (voltage) -> this.sysidElevatorRunVoltage(voltage.in(Volts)),
    null, // No log consumer, since data is recorded by URCL
    this)
    );
}
public void sysidElevatorRunVoltage(double V)
{
  // if (CanMoveElevatorUp() && V > 0)
  //   ElevatorStageMotor.setVoltage(V);
  // else if (CanMoveElevatorDown() && V < 0)
  //   ElevatorStageMotor.setVoltage(V);
  // else
  //   ElevatorStageMotor.setVoltage(0);
}
public  SysIdRoutine ElbowSysIDRun(Config config)
{
  return new SysIdRoutine(
  config,
   new SysIdRoutine.Mechanism(
    (voltage) -> this.sysidElbowRunVoltage(voltage.in(Volts)),
    null, // No log consumer, since data is recorded by URCL
    this)
    );
}
public void sysidElbowRunVoltage(double V)
{
  // if (CanMoveElbowInc() && V > 0)
  //   ElbowMotor.setVoltage(V);
  // else if (CanMoveElbowDec() && V < 0)
  //   ElbowMotor.setVoltage(V);
  // else
  //   ElbowMotor.setVoltage(0);
}
public  SysIdRoutine WristSysIDRun(Config config)
{
  return new SysIdRoutine(
  config,
   new SysIdRoutine.Mechanism(
    (voltage) -> this.sysidWristRunVoltage(voltage.in(Volts)),
    null, // No log consumer, since data is recorded by URCL
    this)
    );
}
public void sysidWristRunVoltage(double V)
{
  // if (CanMoveWristInc() && V > 0) 
  //   WristMotor.setVoltage(V);
  // else if (CanMoveWristDec() && V < 0)
  //   WristMotor.setVoltage(V);
  // else
  //   WristMotor.setVoltage(0);
}

public void LogElevatorData()
{

}

  private void readSensorValues() {
    UpStop = ElevatorUpStop.get();
    DownStop = ElevatorDownStop.get();
    CoralPresent = ElevatorCoralPresence.get();

    ElbowAngle = ElbowAbsoluteEncoder.getPosition();
    WristAngle = WristAbsoluteEncoder.getPosition();

    LaserCan.Measurement measurement = LaserCan.getMeasurement();
    

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      elevatorHeightMM = measurement.distance_mm;
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }

}

