// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotPositions.Climb;
import frc.robot.Constants.RobotPositions.Level2;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_SetArmforClimb extends Command {
  /** Creates a new CmdT_SetArmforClimb. */
  CoralElevatorSubsystem CoralSS;
  public CmdT_SetArmforClimb(CoralElevatorSubsystem css) {
    // Use addRequirements() here to declare subsystem dependencies.
    CoralSS = css;
    addRequirements(CoralSS);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CoralSS.DH_Out_IsClimbing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CoralSS.setArmPosition(Climb.height, Climb.elbow, Climb.wrist);
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