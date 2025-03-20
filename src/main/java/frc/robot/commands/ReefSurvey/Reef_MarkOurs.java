// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefSurvey;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reef_MarkOurs extends Command {
  /** Creates a new Reef_MarkOurs. */
  ReefSurveySubsystem Reef_SS;
  int Rseg = 0;
  int Rside = 0;
  int Rlevel = 0;
  public Reef_MarkOurs(ReefSurveySubsystem  reefss, int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    Reef_SS = reefss;
    
    Rside = side ; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rlevel = Reef_SS.DH_In_ScoreSelection-1;
    Rseg = Reef_SS.DH_In_ReefSegment;
    //System.out.println ("Reef score ls"+Rlevel + " " +Rseg);
    SmartDashboard.putNumber("Rlevel", Rlevel);
    SmartDashboard.putNumber("Rseg", Rseg);
    //pos 0 to 11
    int pos = Rside + Rseg*2;
    if (pos < 12 && pos >=0 && Rlevel >= 0 && Rlevel <= 3)
   {  //System.out.println ("Reef score lp"+Rlevel + " " +pos);
     //call three times to set threshold high enough
     Reef_SS.ScoreReef(Rlevel, pos);
     Reef_SS.ScoreReef(Rlevel, pos);
     Reef_SS.ScoreReef(Rlevel, pos);
     SmartDashboard.putNumber("Scorepos", pos);
     
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
