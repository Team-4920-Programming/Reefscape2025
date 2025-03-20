// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climber.TeleOp.CmdT_ClimberOut;

import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToTarget;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

import frc.robot.subsystems.DataHighway.DataHighwaySubsystem.Target;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SeqClimbV2 extends SequentialCommandGroup {
  /** Creates a new Cmd_SeqRemoveAlgea. */

  public Cmd_SeqClimbV2(SwerveSubsystem Drive_SS, ClimberSubsystem Climb_SS, CoralElevatorSubsystem Coral_SS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addRequirements(Drive_SS, Coral_SS);
      // addCommands(
      // new ParallelCommandGroup(new CmdT_DriveToPose(Drive_SS,new Pose2d(10.438,2.952, new Rotation2d((Units.degreesToRadians(0)))),2), new CmdT_ClimberOut(Climb_SS), new CmdT_WristEject(Coral_SS))//,
      // new CmdT_DriveToPose(Drive_SS,new Pose2d(8.87,5.1, new Rotation2d(Units.degreesToRadians(180))),0.75),
      // new CmdT_DriveToPose(Drive_SS,new Pose2d(8.07,5.1, new Rotation2d(Units.degreesToRadians( 180))),0.75)
      // new CmdT_RunClimberIn4Seq(Climb_SS))
      // );
    addCommands(//7.11,5.1 180 - Blue
    //10.438,2.952 0 - Red
    //17.548 8.052 - Field
      new ParallelCommandGroup(new CmdT_DriveToTarget(Drive_SS, Target.SELECTEDCLIMB, 2, 0), new CmdT_ClimberOut(Climb_SS))//, new CmdT_WristEject(Coral_SS)),
      // new CmdT_DriveToPoseRelative(Drive_SS, 1.25, Units.metersToInches(-1.76), 0, 0),
      // new CmdT_DriveToPoseRelative(Drive_SS, 1.25, Units.metersToInches(0.87), 0, 0)

    );
  }
}
