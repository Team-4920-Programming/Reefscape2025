// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import org.dyn4j.geometry.Rotation;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignCoralFeederStation;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_DriveToFeederPosition_Relative extends Command {
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  Pose2d startingPose;
  double minSpeed = 0.25;
  boolean inter;
  Twist2d RobotDelta;
  double xytolerance = 0.05;
  double thetaTolerance = Units.degreesToRadians(5);
  double px = 2;
  double py = 2;
  double pt = 5;
  public CmdA_DriveToFeederPosition_Relative(SwerveSubsystem DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    inter = false;
    Pose2d targetAprilTagPose = DriveSS.GetClosestPickupSlot();

    targetPose = new Pose2d();
    targetPose = targetAprilTagPose;
      Transform2d test = new Transform2d(AutoAlignCoralFeederStation.distanceFromFace, 0
      ,new Rotation2d(0));

      targetPose = DriveSS.GetClosestPickupSlot().plus(test);


    
    DogLog.log("Auto/DriveToFeederRelative/CommandStatus", "initialized");
    DogLog.log("Auto/DriveToFeederRelative/Init/targetPose", targetPose);

    startingPose = DriveSS.getPose();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Auto/DriveToFeederRelative/CommandStatus", "executing");
    DogLog.log("Auto/DriveToFeederRelative/Exec/CurrentRobotPose", DriveSS.getPose());

    Pose2d currentPose = DriveSS.getPose();

    RobotDelta = currentPose.log(targetPose);

    DogLog.log("Auto/DriveToFeederRelative/Exec/TwistDx", RobotDelta.dx);
    DogLog.log("Auto/DriveToFeederRelative/Exec/TwistDy", RobotDelta.dy);
    DogLog.log("Auto/DriveToFeederRelative/Exec/TwistDtheta", Units.radiansToDegrees(RobotDelta.dtheta));

      double XVel = 0;
      double YVel = 0;
      double RotVel = 0;
      double RawXVel = 0;
      double RawYVel = 0;
      double RawRotVel = 0;
      RawXVel = RobotDelta.dx * px;
      RawYVel = RobotDelta.dy * py;
      RawRotVel = RobotDelta.dtheta * pt;
      XVel = RawXVel;
      YVel = RawYVel;
      RotVel = RawRotVel;

    XVel = MathUtil.clamp(XVel, -3, 3);
    YVel = MathUtil.clamp(YVel, -3,3);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    DogLog.log("Auto/DriveToFeederRelative/Exec/ClampedXPIDOutput", XVel);
    DogLog.log("Auto/DriveToFeederRelative/Exec/ClampedYPIDOutput", YVel);
    DogLog.log("Auto/DriveToFeederRelative/Exec/ClampedRotPIDOutput", RotVel);

    if (!atSetpoint(XVel, xytolerance)){
      if (XVel > 0 ){
        XVel = Math.max(XVel, minSpeed);
      }
      else if (XVel < 0 ){
        XVel = Math.min(XVel, -minSpeed);
      }
    }
    if (atSetpoint(XVel, xytolerance)){
      XVel = RawXVel;
    }
    if (atSetpoint(YVel, xytolerance)){
      YVel = RawYVel ;
    }
    if (atSetpoint(RotVel, thetaTolerance)){
      RotVel = RawRotVel ;
    }
    if (!atSetpoint(YVel, xytolerance)){
      if (  YVel > 0 ){
        YVel = Math.max(YVel, minSpeed);
      }
      else if (YVel < 0 ){
        YVel = Math.min(YVel, -minSpeed);
      }
    }
    

    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,false);


    DogLog.log("Auto/DriveToFeederRelative/Exec/ActualXOutput", XVel);
    DogLog.log("Auto/DriveToFeederRelative/Exec/ActualYOutput", YVel);
    DogLog.log("Auto/DriveToFeederRelative/Exec/ActualRotOutput", RotVel);
    DogLog.log("Auto/DriveToFeederRelative/Check/XPIDAtSetpoint", atSetpoint(RobotDelta.dx, xytolerance));
    DogLog.log("Auto/DriveToFeederRelative/Check/YPIDAtSetpoint", atSetpoint(RobotDelta.dy, xytolerance));
    DogLog.log("Auto/DriveToFeederRelative/Check/RotPIDAtSetpoint", atSetpoint(RobotDelta.dtheta, thetaTolerance));


    
   
    //System.out.println("Driving toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Auto/DriveToFeederRelative/CommandStatus", "finished");
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    //System.out.println("DrivetoReef Finished");
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    DogLog.log("Auto/DriveToFeederRelative/End/Interrupted", inter); 
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint(RobotDelta.dx, xytolerance) && atSetpoint(RobotDelta.dy, xytolerance) && atSetpoint(RobotDelta.dtheta, thetaTolerance);
  }

  public boolean wasInterrupted(){
    return inter;
  }

  public boolean atSetpoint(double error, double tolerance){
    return Math.abs(error) <= tolerance;
  }
}
