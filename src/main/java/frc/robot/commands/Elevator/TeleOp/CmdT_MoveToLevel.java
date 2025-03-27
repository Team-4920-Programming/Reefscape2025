// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.Constants.RobotPositions.*;

import frc.robot.commands.Elevator.TeleOp.CmdT_Level1;
import frc.robot.commands.Elevator.TeleOp.CmdT_Level2;
import frc.robot.commands.Elevator.TeleOp.CmdT_Level3;
import frc.robot.commands.Elevator.TeleOp.CmdT_Level4;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_MoveToLevel extends Command {
  /** Creates a new CmdT_MoveToLevel. */
    CoralElevatorSubsystem Coral_SS;
    boolean toggleRedZoneOverride = false;
  int ScoreLevel =0;
  double TargetHeight = 0;
  double TargetElbowAng = 0;
  double TargetWristAng = 0;
  public CmdT_MoveToLevel(CoralElevatorSubsystem CoralSS, boolean b) {
    // Use addRequirements() here to declare subsystem dependencies.
    Coral_SS  = CoralSS;
    toggleRedZoneOverride = b;
    addRequirements(CoralSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Coral_SS.setIsScoring(true);
      DogLog.log("Tele/MoveToLevelCmd/CommandStatus", "initialized");
      Coral_SS.OverrideRedZone = toggleRedZoneOverride;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DogLog.log("Tele/MoveToLevelCmd/CommandStatus", "executing");    
    //System.out.println("Moving to Level");
    if (!Coral_SS.DH_In_RedZone || Coral_SS.OverrideRedZone)
    {
      //System.out.println("Moving to Level (In yellow)");
      if (Coral_SS.GetScoreSelection() ==1)
      {
        DogLog.log("Tele/MoveToLevelCmd/Conditions/ScoreSelection", 1);
        //System.out.println("Moving to Level 1");
        //CmdT_Level1 GoLevel1 = new CmdT_Level1(Coral_SS);
        //GoLevel1.schedule();
        TargetHeight = Level1.height;
        TargetElbowAng = Level1.elbow;
        TargetWristAng = Level1.wrist;
      }
      if (Coral_SS.GetScoreSelection() ==2)
      {
        DogLog.log("Tele/MoveToLevelCmd/Conditions/ScoreSelection", 2);
        //System.out.println("Moving to Level 2");
        //dT_Level2 GoLevel2 = new CmdT_Level2(Coral_SS);
        //GoLevel2.schedule();
        TargetHeight = Level2.height;
        TargetElbowAng = Level2.elbow;
        TargetWristAng = Level2.wrist;
      }
      if (Coral_SS.GetScoreSelection() ==3)
      {
        DogLog.log("Tele/MoveToLevelCmd/Conditions/ScoreSelection", 3);
        //System.out.println("Moving to Level 3");
        //CmdT_Level3 GoLevel3 = new CmdT_Level3(Coral_SS);
        //GoLevel3.schedule();
        TargetHeight = Level3.height;
        TargetElbowAng = Level3.elbow;
        TargetWristAng = Level3.wrist;
      }
      if (Coral_SS.GetScoreSelection() ==4)
      {
        DogLog.log("Tele/MoveToLevelCmd/Conditions/ScoreSelection", 4);
        //System.out.println("Moving to Level 4");
        //CmdT_Level4 GoLevel4 = new CmdT_Level4(Coral_SS);
        //GoLevel4.schedule();
        TargetHeight = Level4.height;
        TargetElbowAng = Level4.elbow;
        TargetWristAng = Level4.wrist;
        // TargetHeight = SmartDashboard.getNumber("ElevatorTestingHeight", 0.725);
        // TargetElbowAng = SmartDashboard.getNumber("ElbowTestingAngle", 180);
        // TargetWristAng = SmartDashboard.getNumber("WristTestingAngle", -33.0);
      }
      DogLog.log("Tele/MoveToLevelCmd/Exec/TargetHeight", TargetHeight);
      DogLog.log("Tele/MoveToLevelCmd/Exec/TargetElbowAng", TargetElbowAng);
      DogLog.log("Tele/MoveToLevelCmd/Exec/TargetWristAng", TargetWristAng);
      Coral_SS.setArmPosition(TargetHeight, TargetElbowAng, TargetWristAng);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    DogLog.log("Tele/MoveToLevelCmd/CommandWasInterrupted", interrupted);
    DogLog.log("Tele/MoveToLevelCmd/CommandStatus", "finished");
    // Coral_SS.OverrideRedZone = false;
    //System.out.println("Move to Level End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    DogLog.log("Tele/MoveToLevelCmd/FinishedCondition/NotInYellowZone", !Coral_SS.DH_In_YellowZone);
    DogLog.log("Tele/MoveToLevelCmd/FinishedCondition/ElevatorAtSetpoint", Coral_SS.IsElevatorAtSetpoint(TargetHeight));
    DogLog.log("Tele/MoveToLevelCmd/FinishedCondition/ElbowAtSetpoint", Coral_SS.IsElbowAtSetpoint(TargetElbowAng));
    DogLog.log("Tele/MoveToLevelCmd/FinishedCondition/WristAtSetpoint", Coral_SS.IsWristAtSetpoint(TargetWristAng));

    return (Coral_SS.IsElevatorAtSetpoint(TargetHeight) && Math.abs(Coral_SS.GetElbowAngle() - TargetElbowAng) <= 20 && Math.abs(Coral_SS.GetWristAngleWorldCoordinates() - TargetWristAng) <= 20 ) || (Coral_SS.DH_In_RedZone && !toggleRedZoneOverride);
  }
}
