// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BoltLog;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_CoralOutTake extends Command {
  /** Creates a new Cmd_Shoot_TeleOp. */
  CoralElevatorSubsystem CoralElevatorSS; 
    private final BoltLog BoltLogger = new BoltLog();
    private Timer PresentTimer = new Timer();
  boolean CoralOut = false;
  double speed = 0.0;
  public CmdT_CoralOutTake(CoralElevatorSubsystem CoralElevator_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    CoralElevatorSS = CoralElevator_Subsystem;
     
    addRequirements(CoralElevatorSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("Tele/CoralOutTakeCmd/CommandStatus", "initialized");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Tele/CoralOutTakeCmd/CommandStatus", "executing");

      // BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "execute", "Executing", true);
      int i = CoralElevatorSS.GetScoreSelection();

      if (i == 1){
        speed = 0.35;
      }
      else{
        speed = 0.75;
      }
  
      DogLog.log("Tele/CoralOutTakeCmd/Exec/IntakeSpeed", -speed);

      CoralElevatorSS.setIntakeSpeed(-speed);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Tele/CoralOutTakeCmd/CommandStatus", "finished");
    DogLog.log("Tele/CoralOutTakeCmd/CommandWasInterrupted", interrupted);

    CoralElevatorSS.setIntakeSpeed(0.0);
    // BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "Execute", "Executing", false);
    DogLog.log("Tele/CoralOutTakeCmd/End/RobotPose",CoralElevatorSS.DH_In_RobotPose);
    DogLog.log("Tele/CoralOutTakeCmd/End/RobotHeading",CoralElevatorSS.DH_In_RobotPose.getRotation().getDegrees());
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
    return (PresentTimer.hasElapsed(.25));
  }
}