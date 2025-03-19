// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Auto;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.Constants.RobotPositions;
import frc.robot.Constants.RobotPositions.CoralStation;
import frc.robot.Constants.RobotPositions.Level2;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_Station extends Command {
  /** Creates a new CmdT_Level1. */
  CoralElevatorSubsystem m_ElevatorSubsystem;
  public CmdA_Station(CoralElevatorSubsystem elevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSS;
    addRequirements(m_ElevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
            DogLog.log("Auto/StationCmd/CommandStatus", "initialized");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    DogLog.log("Auto/StationCmd/CommandStatus", "executing");
        m_ElevatorSubsystem.setArmPosition(CoralStation.height, CoralStation.elbow, CoralStation.wrist);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Auto/StationCmd/CommandStatus", "finished");
    DogLog.log("Auto/StationCmd/CommandWasInterrupted", interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ElevatorSubsystem.IsElbowAtSetpoint() && Math.abs(m_ElevatorSubsystem.getFilteredElevatorHeight() - CoralStation.height) <= 0.1 && m_ElevatorSubsystem.IsWristAtSetpoint() ;
  }
}
