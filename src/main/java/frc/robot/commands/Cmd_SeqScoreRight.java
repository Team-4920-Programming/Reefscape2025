// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.TeleOp.*;
import frc.robot.commands.ReefSurvey.Reef_MarkOurs;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToReefPosition;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToReefPositionV2;
import frc.robot.commands.swervedrive.TeleOp.CmdT_EnableAutoAim;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV2;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SeqScoreRight extends SequentialCommandGroup {
  /** Creates a new Cmd_SeqRemoveAlgea. */
  public Cmd_SeqScoreRight(CoralElevatorSubsystem Coral_SS, SwerveSubsystem Drive_SS,ReefSurveySubsystem Reef_SS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CmdT_MoveToLevel(Coral_SS),
      new CmdT_DriveToReefPositionV2(Drive_SS,2).withTimeout(7),
      new CmdT_CoralOutTake(Coral_SS),
      new Reef_MarkOurs(Reef_SS, 1),
      new CmdT_DriveToReefPosition(Drive_SS,4),
      new CmdT_Station(Coral_SS),
      new CmdT_EnableAutoAim(Drive_SS));
  }
}
