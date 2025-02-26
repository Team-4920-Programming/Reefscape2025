// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefSurvey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefTracking extends Command {
  /** Creates a new ReefTracking. */
  ReefSurveySubsystem ReefSS;
  DataHighwaySubsystem DHSS;
  public ReefTracking(ReefSurveySubsystem reefss, DataHighwaySubsystem datahighway) {
    // Use addRequirements() here to declare subsystem dependencies.
    ReefSS = reefss;
    DHSS = datahighway;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //grab the robot pose and camera directon
    //identify which section of the reef we are looking at
    //use the limelight height and the target x,y to determine which coral we are looking at.
    //use ReefsSS to add 1 to the reef scoring.
    //Once it hits 3 we can filter out that location in the targeting logic

    ReefSS.ScoreReef(1, 3);
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
