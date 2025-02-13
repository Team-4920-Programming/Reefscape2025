// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BoltLog;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static edu.wpi.first.units.Units.Newton;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;

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
  SparkMax ElbowMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxSim ElbowMotorSim = new SparkMaxSim(ElbowMotor, ElbowGearbox);

  DCMotor WristGearbox = DCMotor.getNEO(1);
  SparkMax WristMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxSim WrstMotorSim = new SparkMaxSim(WristMotor, WristGearbox);

  SingleJointedArmSim m_armSim = new SingleJointedArmSim(ElbowGearbox, 0, 0, 0, 0, 0, false, 0, null) 
  ElevatorSim m_ElevSim = new ElevatorSim(null, ElbowGearbox, 0, 0, false, 0, null)

  ElevatorFeedforward ElevFF = new ElevatorFeedforward(0, 0, 0)
  ArmFeedforward ArmFF = new ArmFeedforward(0, 0, 0);
  
  public CoralElevatorSubsystem() {
    if (Robot.isSimulation())
    {
      SimElevatorRoot = SimElevator.getRoot("elevator", 0.5, 0);
      Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 0.9, 90));
      Elbow = Stage1.append(new MechanismLigament2d("Elbow",0.5,170));
      Wrist = Elbow.append(new MechanismLigament2d("Wriat",0.2,90));
    }
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
  
}

