// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import frc.robot.commands.Elevator.TeleOp.*;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToAlgaeRemoval;

import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToPoseRelative;

import frc.robot.commands.swervedrive.TeleOp.CmdT_StopDrive;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SeqRemoveLowAlgea extends SequentialCommandGroup {
  /** Creates a new Cmd_SeqRemoveAlgea. */
  public Cmd_SeqRemoveLowAlgea(CoralElevatorSubsystem Coral_SS, SwerveSubsystem Drive_SS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CmdT_StopDrive(Drive_SS),
      new ParallelDeadlineGroup(new CmdT_DriveToAlgaeRemoval(Drive_SS,4),new CmdT_AlgaeL2(Coral_SS)),
      new CmdT_DriveToPoseRelative(Drive_SS,4,0,15.5,0.0).withTimeout(1.5),
      new CmdT_DriveToPoseRelative(Drive_SS,4,-36,0.0,0.0).withTimeout(2.0)

      );
  }
}
