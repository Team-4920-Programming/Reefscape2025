// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefSurvey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reef_MarkOurs extends Command {
  /** Creates a new Reef_MarkOurs. */
  ReefSurveySubsystem Reef_SS;
  int Rseg = 0;
  int Rside = 0;
  int Rlevel = 0;
  public Reef_MarkOurs(ReefSurveySubsystem  reefss,int Segment, int side, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    Reef_SS = reefss;
    Rseg = Segment;
    Rside = side;
    Rlevel = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int pos = Rside + Rseg *6;
    if (pos < 12 && Rlevel < 3)
   {
     //call three times to set threshold high enough
     Reef_SS.ScoreReef(Rlevel, pos);
     Reef_SS.ScoreReef(Rlevel, pos);
     Reef_SS.ScoreReef(Rlevel, pos);
   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
