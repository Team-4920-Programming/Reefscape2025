// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_CheckSetpoints extends Command {

  CoralElevatorSubsystem CoralSS;
  /** Creates a new CmdT_CheckSetpoints. */
  public CmdT_CheckSetpoints(CoralElevatorSubsystem css) {
    CoralSS = css;
    addRequirements(CoralSS);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!CoralSS.DH_Out_MechAtGoal)
      CoralSS.OverrideRedZone = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  //   if (CoralSS.DH_Out_MechAtGoal)
  //     CoralSS.OverrideRedZone = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralSS.OverrideRedZone = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralSS.DH_Out_MechAtGoal ;//&& CoralSS.OverrideRedZone == false;
  }
}
