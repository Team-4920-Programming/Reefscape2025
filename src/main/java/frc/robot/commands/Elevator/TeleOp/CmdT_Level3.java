// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.Constants.RobotPositions;
import frc.robot.Constants.RobotPositions.Level2;
import frc.robot.Constants.RobotPositions.Level3;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_Level3 extends Command {
  /** Creates a new CmdT_Level1. */
  CoralElevatorSubsystem m_ElevatorSubsystem;
  public CmdT_Level3(CoralElevatorSubsystem elevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSS;
    addRequirements(m_ElevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.setIsScoring(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        m_ElevatorSubsystem.setArmPosition(Level3.height, Level3.elbow, Level3.wrist);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_ElevatorSubsystem.IsElevatorAtSetpoint(Level3.height) && m_ElevatorSubsystem.IsWristAtSetpoint(Level3.wrist) && m_ElevatorSubsystem.IsElbowAtSetpoint(Level3.elbow));
  }
}
