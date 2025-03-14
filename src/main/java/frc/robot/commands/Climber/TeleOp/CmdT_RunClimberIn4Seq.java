// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_RunClimberIn4Seq extends Command {
  /** Creates a new CmdT_ClimberOut. */
  ClimberSubsystem Climber_SS;
  public CmdT_RunClimberIn4Seq(ClimberSubsystem Climber) {
    Climber_SS = Climber;
    addRequirements(Climber_SS);
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Climber_SS.GetClimberEncoder() > 85){
      Climber_SS.RunClimberIn(0.7);
    }
    else{
      Climber_SS.StopClimber();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber_SS.StopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
