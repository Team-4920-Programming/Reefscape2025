// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.TeleOp.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToReefPosition;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SeqRemoveAlgea extends SequentialCommandGroup {
  /** Creates a new Cmd_SeqRemoveAlgea. */
  public Cmd_SeqRemoveAlgea(CoralElevatorSubsystem Coral_SS, SwerveSubsystem Drive_SS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new CmdT_AlgaeLowApproach(Coral_SS),
      new CmdT_DriveToReefPosition(Drive_SS,3),
      new CmdT_AlgaeLowRetract(Coral_SS),
      new CmdT_DriveToReefPosition(Drive_SS,4)

      );
  }
}
