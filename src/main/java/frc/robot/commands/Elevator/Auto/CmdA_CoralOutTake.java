// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BoltLog;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_CoralOutTake extends Command {
  /** Creates a new Cmd_Shoot_TeleOp. */
  CoralElevatorSubsystem CoralElevatorSS; 
    private final BoltLog BoltLogger = new BoltLog();
    private Timer PresentTimer = new Timer();
  boolean CoralOut = false;
  public CmdA_CoralOutTake(CoralElevatorSubsystem CoralElevator_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    CoralElevatorSS = CoralElevator_Subsystem;
     
    addRequirements(CoralElevatorSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "execute", "Executing", true);
    if (Robot.isSimulation()){
                
    }
  
      CoralElevatorSS.setIntakeSpeed(-1.0);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralElevatorSS.setIntakeSpeed(0.0);
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "Execute", "Executing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!CoralElevatorSS.isCoralPresent() && !PresentTimer.isRunning())
        PresentTimer.start();
    if (CoralElevatorSS.isCoralPresent())
    {
        PresentTimer.stop();
        PresentTimer.reset();
    }
    return (PresentTimer.hasElapsed(.1));
  }
}