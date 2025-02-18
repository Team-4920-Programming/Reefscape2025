// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DIO;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.RobotLimits;
import frc.robot.Robot;

import au.grapplerobotics.LaserCan;
import dev.doglog.DogLog;
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

  SparkMax ElevatorStageMotor = new SparkMax(CanIDs.CoralElevator.ElevatorStage, MotorType.kBrushless);
  SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  SparkMax ElbowMotor = new SparkMax(CanIDs.CoralElevator.Elbow, MotorType.kBrushless);
  SparkMax WristMotor = new SparkMax(CanIDs.CoralElevator.Wrist, MotorType.kBrushless);
  SparkMax CoralIntakeMotor = new SparkMax(CanIDs.CoralElevator.CoralIntake, MotorType.kBrushless);


  // PIDs

  ElevatorFeedforward ElevFF = new ElevatorFeedforward(Elevator.ks, Elevator.kg, Elevator.kv,Elevator.ka);
  ArmFeedforward ElbowFF = new ArmFeedforward(PIDs.CoralElevator.Elbow.ks, PIDs.CoralElevator.Elbow.kg, PIDs.CoralElevator.Elbow.kv);
  ProfiledPIDController ElevatorPID = new ProfiledPIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd, new Constraints(PIDs.CoralElevator.Elevator.maxVelocity,PIDs.CoralElevator.Elevator.maxAcceleration));
  // PIDController ElevatorPID = new PIDController(PIDs.CoralElevator.Elevator.kp, PIDs.CoralElevator.Elevator.ki, PIDs.CoralElevator.Elevator.kd);
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


  //SysID nonsense

  public final Trigger atMin = new Trigger(() -> !CanMoveElevatorDown());
  public final Trigger atMax = new Trigger(() -> !CanMoveElevatorUp());
  



  public CoralElevatorSubsystem() {

    ElevatorPID.setTolerance(0.05);
    // ElevatorPID.setGoal(Constants.RobotLimits.Elevator.offset);
    ElevatorPID.setGoal(new State(Constants.RobotLimits.Elevator.offset, 0));
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.inverted(false);
    ElevatorStageMotor.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    if (Robot.isSimulation())
   {
      SimElevatorRoot = SimElevator.getRoot("elevator", 0.5, 0);
      Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
      Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
      Wrist = Elbow.append(new MechanismLigament2d("Wrist",0.2,90));
    }
  }
  public void SetElevatorPosition(double height)
{
  if (height > ElevatorPID.getGoal().position && CanMoveElevatorUp()){
    ElevatorPID.setGoal(new State(height, 0));
  }
  else if (height < ElevatorPID.getGoal().position && CanMoveElevatorDown()){
    ElevatorPID.setGoal(new State(height, 0));
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

public boolean IsElevatorAtSetpoint(){
  return ElevatorPID.atSetpoint();
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
    // elevatorOutput = ElevatorPID.calculate(elevatorHeightMM) + ElevFF.calculate(PIDs.CoralElevator.Elevator.maxVelocity)/RobotController.getBatteryVoltage();
    double elevatorPIDValue = ElevatorPID.calculate(getHeightLaserMeters());
    double linearVelocity = getVelocityMetersPerSecond();
    double elevatorFFValue = ElevFF.calculate(linearVelocity);
    elevatorOutput = MathUtil.clamp(elevatorPIDValue + elevatorFFValue,-7,7);
    
    elbowOutput = ElbowPID.calculate(ElbowAngle) + Math.toDegrees(ElbowFF.calculate (Math.toRadians(ElbowAbsoluteEncoder.getPosition()), PIDs.CoralElevator.Elbow.maxVelocity));
    wristOutput = WristPID.calculate(WristAngle);
    SmartDashboard.putNumber("Elevator PID Output", elevatorPIDValue);
    SmartDashboard.putNumber("Elevator FF Output", elevatorFFValue);
    SmartDashboard.putNumber("Elevator Setpount", ElevatorPID.getGoal().position);
    SmartDashboard.putNumber("Elevator Output(V)", elevatorOutput);
    SmartDashboard.putNumber("ElevatorVelocity", linearVelocity);
    SmartDashboard.putBoolean("CanMoveElevatorUp", CanMoveElevatorUp());
    SmartDashboard.putBoolean("CanMoveElevatorDown", CanMoveElevatorDown());
    SmartDashboard.putBoolean("ElevatorAtGoal", ElevatorPID.atGoal());
    if ((CanMoveElevatorUp() && elevatorOutput > 0 && !ElevatorPID.atGoal()) || (CanMoveElevatorDown() && elevatorOutput < 0 && !ElevatorPID.atGoal()))
    {

      ElevatorStageMotor.setVoltage(elevatorOutput);

    }
    // else{
    //   System.out.println("Here");
    //   ElevatorStageMotor.setVoltage(0);
    // }
      
    // ElbowMotor.set(elbowOutput);
    // WristMotor.set(wristOutput);

  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation())
    SmartDashboard.putData  ("Simulation/Elevator",SimElevator);

  }


  private void readSensorValues() {
    UpStop = !ElevatorUpStop.get();
    DownStop = !ElevatorDownStop.get();
    CoralPresent = ElevatorCoralPresence.get();

    ElbowAngle = ElbowAbsoluteEncoder.getPosition();
    WristAngle = WristAbsoluteEncoder.getPosition();

    LaserCan.Measurement measurement= LaserCan.getMeasurement();


    

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      elevatorHeightMM = measurement.distance_mm;
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
    // System.out.println("Elevator Height = " + elevatorHeightMM);
    
  }

  // SysID nonsense

  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  private final MutAngle          m_rotations      = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);

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
    System.out.println("Normalized V = " + (V / RobotController.getBatteryVoltage()));
    System.out.println("V = " + V);
    System.out.println("CanMoveElevatorUp = " + CanMoveElevatorUp());
    System.out.println("CanMoveElevatorDown = " + CanMoveElevatorDown());
    if (CanMoveElevatorUp() && V > 0)
    ElevatorStageMotor.set(V / RobotController.getBatteryVoltage());
    else if (CanMoveElevatorDown() && V < 0)
    ElevatorStageMotor.set(V / RobotController.getBatteryVoltage());
    else
    ElevatorStageMotor.set(0);
  }

  public Command sysIDElevatorAll(){
    return (ElevatorSysID.dynamic(Direction.kForward).until(atMax)
        .andThen(ElevatorSysID.dynamic(Direction.kReverse).until(atMin))
        .andThen(ElevatorSysID.quasistatic(Direction.kForward).until(atMax))
        .andThen(ElevatorSysID.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE")));
  }

  public Command sysIdQuasistaticElevator(SysIdRoutine.Direction direction) {
    return ElevatorSysID.quasistatic(direction);
  }

  public Command sysIdDynamicElevator(SysIdRoutine.Direction direction) {
    return ElevatorSysID.dynamic(direction);
  }


  //Elbow

  private final SysIdRoutine ElbowSysID = new SysIdRoutine
  (new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism(
    (voltage) -> this.sysidElbowRunVoltage(voltage.in(Volts)),
    log -> {
      DogLog.log("SysID/Elbow/VoltageApplied", ElbowMotor.getAppliedOutput() * ElbowMotor.getBusVoltage());
      DogLog.log("SysID/Elbow/Position", ElbowAbsoluteEncoder.getPosition());
      DogLog.log("SysID/Elbow/Velocity", ElbowAbsoluteEncoder.getVelocity());
    },
    this));

  public void sysidElbowRunVoltage(double V)
  {
    if (CanMoveElevatorUp() && V > 0)
    ElbowMotor.setVoltage(V / RobotController.getBatteryVoltage());
    else if (CanMoveElevatorDown() && V < 0)
    ElbowMotor.setVoltage(V / RobotController.getBatteryVoltage());
    else
    ElbowMotor.setVoltage(0);
  }

  

  public Command sysIdQuasistaticElbow(SysIdRoutine.Direction direction) {
    return ElbowSysID.quasistatic(direction);
  }

  public Command sysIdDynamicElbow(SysIdRoutine.Direction direction) {
    return ElbowSysID.dynamic(direction);
  }

  //Wrist

  private final SysIdRoutine WristSysID = new SysIdRoutine
  (new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism(
    (voltage) -> this.sysidWristRunVoltage(voltage.in(Volts)),
    log -> {
      DogLog.log("SysID/Wrist/VoltageApplied", WristMotor.getAppliedOutput() * WristMotor.getBusVoltage());
      DogLog.log("SysID/Wrist/Position", WristAbsoluteEncoder.getPosition());
      DogLog.log("SysID/Wrist/Velocity", WristAbsoluteEncoder.getVelocity());
    },
    this));

  public void sysidWristRunVoltage(double V)
  {
    if (CanMoveElevatorUp() && V > 0)
    WristMotor.setVoltage(V / RobotController.getBatteryVoltage());
    else if (CanMoveElevatorDown() && V < 0)
    WristMotor.setVoltage(V / RobotController.getBatteryVoltage());
    else
    WristMotor.setVoltage(0);
  }

  

  public Command sysIdQuasistaticWrist(SysIdRoutine.Direction direction) {
    return WristSysID.quasistatic(direction);
  }

  public Command sysIdDynamicWrist(SysIdRoutine.Direction direction) {
    return WristSysID.dynamic(direction);
  }

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
    double elevatorHeight = measurement.distance_mm / 1000.0;
    SmartDashboard.putNumber("Elevator Height", elevatorHeight);
    return elevatorHeight;
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //   elevatorHeightMM = measurement.distance_mm;
    // } else {
    //   System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    // }
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


}

