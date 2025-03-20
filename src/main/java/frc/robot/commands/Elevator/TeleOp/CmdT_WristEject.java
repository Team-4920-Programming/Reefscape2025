// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

import frc.robot.Constants.RobotPositions.CoralStation;

import frc.robot.Constants.RobotPositions.WristEject;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_WristEject extends Command {
  /** Creates a new CmdT_Level1. */
  CoralElevatorSubsystem m_ElevatorSubsystem;
  boolean hasCoral;
  Timer t = new Timer();
  public CmdT_WristEject(CoralElevatorSubsystem elevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSS;
    addRequirements(m_ElevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.isSimulation()){

        t.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        m_ElevatorSubsystem.setArmPosition(WristEject.height, WristEject.elbow, WristEject.wrist);

        if (Robot.isSimulation() && t.get() > 3){
          t.stop();
          hasCoral = false;
      }

        if(m_ElevatorSubsystem.IsWristAtSetpoint()){
            m_ElevatorSubsystem.setIntakeSpeed(-1.0);
        }
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.setArmPosition(CoralStation.height, CoralStation.elbow, CoralStation.wrist);
    m_ElevatorSubsystem.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Robot.isSimulation()){
      return hasCoral;
    }
    else{
    }
    return !m_ElevatorSubsystem.DH_Out_HasCoral;
  }
}
