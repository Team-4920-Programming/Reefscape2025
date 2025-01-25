// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralElevator;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BoltLog;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import java.lang.Math;
import frc.robot.BoltLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class CoralElevatorSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final BoltLog BoltLogger = new BoltLog();
  private Mechanism2d SimElevator = new Mechanism2d(80, 80);
  private MechanismRoot2d SimElevatorRoot;
  private MechanismLigament2d Stage1;
  private MechanismLigament2d Stage2;
  public CoralElevatorSubsystem() {
    if (Robot.isSimulation())
    {
      SimElevatorRoot = SimElevator.getRoot("elevator", 4, 0);
      Stage1 = SimElevatorRoot.append(new MechanismLigament2d("Stage1", 30, 30));
      Stage2 = Stage1.append(new MechanismLigament2d("Stage2",30,-45));
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

