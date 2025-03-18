// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Auto;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.Constants.RobotPositions;
import frc.robot.Constants.RobotPositions.Level2;
import frc.robot.Constants.RobotPositions.Level3;
import frc.robot.Constants.RobotPositions.Level4;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_Level4 extends Command {
  /** Creates a new CmdT_Level1. */
  CoralElevatorSubsystem m_ElevatorSubsystem;
  public CmdA_Level4(CoralElevatorSubsystem elevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSS;
    addRequirements(m_ElevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        DogLog.log("Auto/Level4Cmd/CommandStatus", "initialized");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Auto/Level4Cmd/CommandStatus", "executing");

        m_ElevatorSubsystem.setArmPosition(Level4.height, Level4.elbow, Level4.wrist);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Auto/Level4Cmd/CommandStatus", "finished");
    DogLog.log("Auto/Level4Cmd/CommandWasInterrupted", interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ElevatorSubsystem.IsElbowAtSetpoint() && m_ElevatorSubsystem.IsElevatorAtSetpoint() && m_ElevatorSubsystem.IsWristAtSetpoint() ;
  }
}
