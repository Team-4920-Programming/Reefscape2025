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
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_DriveToReefPositionV3_Relative extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  Pose2d startingPose;
  double minSpeed = 0.25;
  boolean inter;
  Twist2d RobotDelta;
  double xytolerance = 0.025;
  double thetaTolerance = Units.degreesToRadians(1);
  double px = 2;
  double py = 2;
  double pt = 5;
  public CmdA_DriveToReefPositionV3_Relative(SwerveSubsystem DriveSubsystem, int Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    branch = Position;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double branchoffset = AutoAlignReef.branchOffset;
    double distanceFromFace = AutoAlignReef.distanceFromFace-Units.inchesToMeters(0.25);
    inter = false;
    Pose2d targetAprilTagPose = DriveSS.GetClosestReefSegment();

    targetPose = new Pose2d();
    targetPose = targetAprilTagPose;
    if (branch == 1){

      Transform2d test = new Transform2d(distanceFromFace, -branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);
      targetPose = targetPose.rotateAround(targetPose.getTranslation(), new Rotation2d(Units.degreesToRadians(180)));

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());

    }
    if (branch == 2){

      Transform2d test = new Transform2d(distanceFromFace, branchoffset + Units.inchesToMeters(1)
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);
      targetPose = targetPose.rotateAround(targetPose.getTranslation(), new Rotation2d(Units.degreesToRadians(180)));

    }
    
    DogLog.log("Auto/DriveToReefRelative/CommandStatus", "initialized");
    DogLog.log("Auto/DriveToReefRelative/Init/Branch", branch);
    DogLog.log("Auto/DriveToReefRelative/Init/targetPose", targetPose);

    startingPose = DriveSS.getPose();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Auto/DriveToReefRelative/CommandStatus", "executing");
    DogLog.log("Auto/DriveToReefRelative/Exec/CurrentRobotPose", DriveSS.getPose());

    Pose2d currentPose = DriveSS.getPose();

    RobotDelta = currentPose.log(targetPose);

    DogLog.log("Auto/DriveToReefRelative/Exec/TwistDx", RobotDelta.dx);
    DogLog.log("Auto/DriveToReefRelative/Exec/TwistDy", RobotDelta.dy);
    DogLog.log("Auto/DriveToReefRelative/Exec/TwistDtheta", Units.radiansToDegrees(RobotDelta.dtheta));

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

    DogLog.log("Auto/DriveToReefRelative/Exec/ClampedXPIDOutput", XVel);
    DogLog.log("Auto/DriveToReefRelative/Exec/ClampedYPIDOutput", YVel);
    DogLog.log("Auto/DriveToReefRelative/Exec/ClampedRotPIDOutput", RotVel);

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


    DogLog.log("Auto/DriveToReefRelative/Exec/ActualXOutput", XVel);
    DogLog.log("Auto/DriveToReefRelative/Exec/ActualYOutput", YVel);
    DogLog.log("Auto/DriveToReefRelative/Exec/ActualRotOutput", RotVel);
    DogLog.log("Auto/DriveToReefRelative/Check/XPIDAtSetpoint", atSetpoint(RobotDelta.dx, xytolerance));
    DogLog.log("Auto/DriveToReefRelative/Check/YPIDAtSetpoint", atSetpoint(RobotDelta.dy, xytolerance));
    DogLog.log("Auto/DriveToReefRelative/Check/RotPIDAtSetpoint", atSetpoint(RobotDelta.dtheta, thetaTolerance));


    
   
    //System.out.println("Driving toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Auto/DriveToReefRelative/CommandStatus", "finished");
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    //System.out.println("DrivetoReef Finished");
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    DogLog.log("Auto/DriveToReefRelative/End/Interrupted", inter); 
    
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
