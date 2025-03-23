// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_IsScoring extends Command {
  /** Creates a new CmdT_CancelScoringSequence. */
  CoralElevatorSubsystem css;
  boolean isScoring;
  public CmdT_IsScoring(CoralElevatorSubsystem CoralSS, boolean s) {
    // Use addRequirements() here to declare subsystem dependencies.
    css = CoralSS;
    isScoring = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    css.setIsScoring(isScoring);
    css.OverrideRedZone = false;

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
