// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotPositions.Level4;
import frc.robot.Constants.RobotPositions.Level4_Far;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_UltrasonicCheck extends Command {
  /** Creates a new CmdT_UltrasonicCheck. */
  CoralElevatorSubsystem css;
  public CmdT_UltrasonicCheck(CoralElevatorSubsystem CoralSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    css = CoralSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("checking ultrasonic");
    int targetScore = css.GetScoreSelection();
    
    if(targetScore == 4){
      if (css.getUltrasonic() >= 10){
        css.OverrideRedZone = true;
        css.setArmPosition(Level4_Far.height, Level4_Far.elbow, Level4_Far.wrist);
      }
      else{
        css.setArmPosition(Level4.height, Level4.elbow, Level4.wrist);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    css.OverrideRedZone = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return css.IsElevatorAtSetpoint() && css.iSWristAtSetpoint() && css.iSElbowAtSetpoint();
  }
}
