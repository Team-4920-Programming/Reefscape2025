// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.Constants.RobotPositions;
import frc.robot.Constants.RobotPositions.Level4;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_Level4 extends Command {
  /** Creates a new CmdT_Level1. */
  CoralElevatorSubsystem m_ElevatorSubsystem;
  public CmdT_Level4(CoralElevatorSubsystem elevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSS;
    addRequirements(m_ElevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //safety Checks
    boolean ElevatorDirectionUp = false;
    boolean ElevatorDirectionDown = false;
    boolean ElbowUp = false;
    boolean ElbowDown = false;
    boolean WristUp = false;
    boolean WristDown = false;

    if (m_ElevatorSubsystem.getHeightMeters() < Level4.height )
    {
      ElevatorDirectionUp = true;
      ElevatorDirectionDown = false;

    }
    if (m_ElevatorSubsystem.getHeightMeters() > Level4.height )
    {
      ElevatorDirectionDown = true;
      ElevatorDirectionUp = false;
    }

    if (m_ElevatorSubsystem.GetElbowAngle() < Level4.elbow )
    {
      ElbowUp = true;
      ElbowDown = false;

    }
    if (m_ElevatorSubsystem.GetElbowAngle() > Level4.elbow )
    {
      ElbowDown = true;
      ElbowUp = false;
    }
    if (m_ElevatorSubsystem.GetWristAngleWorldCoordinates() < Level4.wrist )
    {
      WristUp = true;
      WristDown = false;

    }
    if (m_ElevatorSubsystem.GetWristAngleWorldCoordinates() > Level4.wrist )
    {
      WristUp = false;
      WristDown = true;

    }

    //Elevator Height Move

    Boolean ElbowClearForElevatorUp = false;
    Boolean ElbowClearForEleavatorDown = false;
    Boolean WristClearForElevatorUp = false;
    Boolean WristClearforElevatorDown = false;

    if (m_ElevatorSubsystem.GetElbowAngle() > 0 && m_ElevatorSubsystem.GetElbowAngle() < 190 )
    {
      ElbowClearForEleavatorDown = true;
      ElbowClearForElevatorUp = true;
    }
    if (m_ElevatorSubsystem.GetWristAngleWorldCoordinates() > -90  && m_ElevatorSubsystem.GetWristAngleWorldCoordinates() < 90 )
    {
      WristClearforElevatorDown  = true;
      WristClearForElevatorUp = true;
    }
    if (ElevatorDirectionUp && ElbowClearForElevatorUp && WristClearForElevatorUp)
    {
        m_ElevatorSubsystem.SetElevatorPosition(Level4.height);
    }
    if (ElevatorDirectionDown && ElbowClearForEleavatorDown && WristClearforElevatorDown )
    {
        m_ElevatorSubsystem.SetElevatorPosition(Level4.height);
    }

    //Elbow Move
    Boolean ElevatorClearForElbowUp = false;
    Boolean ElevatorClearForElbowDown = false;
    if (m_ElevatorSubsystem.getHeightMeters() >0 && m_ElevatorSubsystem.getHeightMeters() < .8)
    {
      if (Level4.elbow >0 && Level4.elbow <190)
      {
        ElevatorClearForElbowUp = true;
        ElevatorClearForElbowDown = true;
      }
    }
    Boolean WristClearforElbowUp = false;
    Boolean WristClearforElbowDown = false;
    if (m_ElevatorSubsystem.GetWristAngleWorldCoordinates() >-85 && m_ElevatorSubsystem.GetWristAngleWorldCoordinates() < 85)
    {
     
        WristClearforElbowUp = true;
        WristClearforElbowDown = true;
      
    }
    if (ElbowUp && ElevatorClearForElbowUp && WristClearforElbowUp)
      m_ElevatorSubsystem.SetElbowAngle(Level4.elbow);
    if (ElbowDown && ElevatorClearForElbowDown && WristClearforElbowDown)
      m_ElevatorSubsystem.SetElbowAngle(Level4.elbow);

    Boolean ElbowClearForWristUp = false;
    Boolean ElbowClearForWristDown = false;
    if (m_ElevatorSubsystem.GetElbowAngle() >0 && m_ElevatorSubsystem.GetElbowAngle() <190)
    {
     ElbowClearForWristUp = true;
      ElbowClearForWristDown = true;
    }
    if (WristUp && ElbowClearForWristUp)
    {
      m_ElevatorSubsystem.SetWristAngle(Level4.wrist);
    }
    if (WristDown && ElbowClearForWristDown)
    {
      m_ElevatorSubsystem.SetWristAngle(Level4.wrist);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
