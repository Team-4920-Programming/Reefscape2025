// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.opencv.features2d.FlannBasedMatcher;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.BoltLog;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static edu.wpi.first.units.Units.Newton;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import static edu.wpi.first.units.Units.Volts;
import java.lang.Math;
import frc.robot.BoltLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class CoralElevatorSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final BoltLog BoltLogger = new BoltLog();
  private Mechanism2d SimElevator = new Mechanism2d(1, 1);
  private MechanismRoot2d SimElevatorRoot;
  private MechanismLigament2d Stage1;
  private MechanismLigament2d Elbow;
  private MechanismLigament2d Wrist;
  DCMotor ElevatorUpDownGearbox = DCMotor.getNEO(1);
  SparkMax ElevatorUPDownMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxSim ElevatorUpDownMotorSim = new SparkMaxSim(ElevatorUPDownMotor, ElevatorUpDownGearbox);
  
  DCMotor ElbowGearbox = DCMotor.getNEO(1);
  SparkMax ElbowMotor = new SparkMax(11, MotorType.kBrushless);
  SparkMaxSim ElbowMotorSim = new SparkMaxSim(ElbowMotor, ElbowGearbox);

  DCMotor WristGearbox = DCMotor.getNEO(1);
  SparkMax WristMotor = new SparkMax(12, MotorType.kBrushless);
  SparkMaxSim WrstMotorSim = new SparkMaxSim(WristMotor, WristGearbox);

  //SingleJointedArmSim m_armSim = new SingleJointedArmSim(ElbowGearbox, 1, 0.1, .1, 0, .5*3.145, false, 0, null) ;
  //ElevatorSim m_ElevSim = new ElevatorSim(null, ElbowGearbox, 0, .25, false, 0, null);
  ElevatorFeedforward ElevFF = new ElevatorFeedforward(0.1, 0.1, 0.1);
  ArmFeedforward ArmFF = new ArmFeedforward(0.1, 0.1, 0.1);

  PIDController ElevatorPID = new PIDController(0.001, 0, 0.01);
  PIDController ElbowPID = new PIDController(0.001, 0, 0.01);
  PIDController WristPID = new PIDController(.001, 0, 0.01);

  



  public CoralElevatorSubsystem() {
    if (Robot.isSimulation())
   {
      SimElevatorRoot = SimElevator.getRoot("elevator", 0.5, 0);
      Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
      Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
      Wrist = Elbow.append(new MechanismLigament2d("Wriat",0.2,90));
    }
  }
  private void SetElevatorPosition(double Height)
{
  if (Height > ElevatorPID.getSetpoint() && CanMoveElevatorUP()){
    ElevatorPID.setSetpoint(Height);
  }
  if (Height < ElevatorPID.getSetpoint() && CanMoveElevatorDown()){
    ElevatorPID.setSetpoint(Height);
  }
}

  private void SetElbowPosition(double Angle)
{
  if (Angle > ElbowPID.getSetpoint() && CanMoveElbowInc()){
    ElbowPID.setSetpoint(Angle);
  }
  if (Angle < ElbowPID.getSetpoint() && CanMoveElbowDec()){
    ElbowPID.setSetpoint(Angle);
  }
}
  private void SetWristPosition(double Angle)
{
  if (Angle > WristPID.getSetpoint() && CanMoveWristInc()){
    WristPID.setSetpoint(Angle);
  }
  if (Angle < WristPID.getSetpoint() && CanMoveWristDec()){
    WristPID.setSetpoint(Angle);
  }
}
  

    

private Boolean CanMoveElevatorUP(){
  return false;
}
private Boolean CanMoveElevatorDown(){
  return false;
}
private Boolean CanMoveElbowInc(){
  return false;
}
private Boolean CanMoveElbowDec(){
  return false;
}
private Boolean CanMoveWristInc(){
  return false;
}
private Boolean CanMoveWristDec(){
  return false;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    


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
    (voltage) -> this.runVolts(voltage.in(Volts)),
    null, // No log consumer, since data is recorded by URCL
    this)
    );
}
public void runVolts(double V)
{
  if (CanMoveElevatorUP() && V>0)
    ElevatorUPDownMotor.setVoltage(V);
  else if (CanMoveElevatorDown() && V<0)
    ElevatorUPDownMotor.setVoltage(V);
  else ElevatorUPDownMotor.setVoltage(0);

}
public void LogElevatorData()
{

}
  
}

