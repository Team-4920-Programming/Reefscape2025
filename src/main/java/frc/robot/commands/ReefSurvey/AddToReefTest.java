// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefSurvey;

import java.util.Random;
import java.util.random.RandomGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AddToReefTest extends Command {
  /** Creates a new AddToReefTest. */
  ReefSurveySubsystem R;
  public AddToReefTest(ReefSurveySubsystem rss) {
    R = rss;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int i = 2;
    int j = 7;
    R.ScoreReef(i, j);

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
