// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.TeleOp;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotPositions.CoralStation;
import frc.robot.Constants.RobotPositions.JustScored;
import frc.robot.Constants.RobotPositions.Level4;
import frc.robot.Constants.RobotPositions.TransportCoralDown;
import frc.robot.Constants.RobotPositions.TransportCoralUp;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_Def_Elevator extends Command {
  /** Creates a new CmdT_Def_Elevator. */
  CoralElevatorSubsystem Coral_SS;
  public CmdT_Def_Elevator(CoralElevatorSubsystem ElevatorSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    Coral_SS =ElevatorSS;
    addRequirements(ElevatorSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("Tele/DefaultElevatorCmd/Status","Initialized");
    DogLog.log("Tele/DefaultElevatorCmd/State",0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Tele/DefaultElevatorCmd/Status","Executing");
    Boolean HasCoral = Coral_SS.isCoralPresent();
    boolean InCoralStationZone = Coral_SS.DH_In_CoralZone;
    boolean InRedZone = Coral_SS.DH_In_RedZone;
    boolean justScored = Coral_SS.getJustScored();
    boolean pablo = Coral_SS.getPableOverride();
    boolean isScoring = Coral_SS.getIsScoring();
    boolean isClimbing = Coral_SS.DH_Out_IsClimbing;
    // if (!HasCoral && !InRedZone && !InCoralStationZone)
    //   Coral_SS.setArmPosition(CoralStation.height, CoralStation.elbow, CoralStation.wrist);
    // if (!HasCoral && InCoralStationZone)
    // {
    //   CmdT_CoralIntake Intake = new CmdT_CoralIntake(Coral_SS);
    //   Intake.schedule();
    // }
    if (!isClimbing){
      if (!isScoring){
        if (!HasCoral && !InRedZone){
            DogLog.log("Tele/DefaultElevatorCmd/State",1);
            Coral_SS.setArmPosition(CoralStation.height, CoralStation.elbow, CoralStation.wrist);
            Coral_SS.setJustScored(false);
        }
        if (HasCoral && !InRedZone){
          if (!pablo){
            if (Coral_SS.GetWristAngleWorldCoordinates() <= 95){
              DogLog.log("Tele/DefaultElevatorCmd/State",2);
            Coral_SS.setArmPosition(TransportCoralUp.height, TransportCoralUp.elbow, TransportCoralUp.wrist);
            }
            else{
              DogLog.log("Tele/DefaultElevatorCmd/State",3);
              Coral_SS.setArmPosition(TransportCoralDown.height, TransportCoralDown.elbow, TransportCoralDown.wrist);
            }
          }
          else{
            DogLog.log("Tele/DefaultElevatorCmd/State",4);
            Coral_SS.setArmPosition(TransportCoralDown.height, TransportCoralDown.elbow, TransportCoralDown.wrist);
          }
        } 
      }
      else{
        if (!HasCoral && InRedZone && justScored && Coral_SS.IsElevatorAtSetpoint(Level4.height)){
          DogLog.log("Tele/DefaultElevatorCmd/State",5);
          Coral_SS.setArmPosition(JustScored.height, JustScored.elbow, JustScored.wrist);
          Coral_SS.setJustScored(false);
      }
        
      }
      if (!HasCoral && InCoralStationZone){
        DogLog.log("Tele/DefaultElevatorCmd/State",6);
        Coral_SS.setArmPosition(CoralStation.height, CoralStation.elbow, CoralStation.wrist);
        Coral_SS.setJustScored(false);
        CmdT_CoralIntake Intake = new CmdT_CoralIntake(Coral_SS);
        Intake.schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Tele/DefaultElevatorCmd/Status","Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
