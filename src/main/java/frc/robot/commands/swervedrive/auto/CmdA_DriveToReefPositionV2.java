// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_DriveToReefPositionV2 extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  PIDController XPID = new PIDController(2.0, 0, 0.02);
  PIDController YPID = new PIDController(2.0, 0, 0.02);
  PIDController RotPID = new PIDController(0.15,0,0);
  double minSpeed = 0.3;
  boolean inter;
  public CmdA_DriveToReefPositionV2(SwerveSubsystem DriveSubsystem, int Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    branch = Position;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double branchoffset = AutoAlignReef.branchOffset;
    double distanceFromFace = AutoAlignReef.distanceFromFace;
    inter = false;
    Pose2d targetAprilTagPose = DriveSS.GetClosestReefSegment();

    // double CenterofReefX = 4.5;
    // double CenterofReefY = 4.0;

    // if (DriveSS.DH_OUT_isRedAlliance)
    // {
    //   CenterofReefX  = 17.5-4.5;
    // }
    targetPose = new Pose2d();
    targetPose = targetAprilTagPose;
    if (branch == 1){

      Transform2d test = new Transform2d(distanceFromFace, -branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());

    }
    if (branch == 2){

      Transform2d test = new Transform2d(distanceFromFace, branchoffset + Units.inchesToMeters(1)
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());
    }
    
  //   double ReefRadius = Units.inchesToMeters(65.5)/2;
  //   //onshape cordinates of blue 
  //   //y along alliance wall 
  //   //x from blue to red
  //   double LeftOffsetAng=-8;
  //   double RightOffsetAng=8;
  //   double RobotOffset = Units.inchesToMeters(15); 
    
  //   double offsetAng = 0;
  //   //System.out.println("Initializing Drive to Reef *****************");
  //   int ReefSegment = DriveSS.getReefSegment();
  //   //System.out.println("ReefSegment" + ReefSegment);
  //   if (Reef_Position ==1)
  //     offsetAng = LeftOffsetAng;
  //   if (Reef_Position == 2)
  //     offsetAng = RightOffsetAng;
  //   if (Reef_Position == 3 || Reef_Position == 4)
  //     offsetAng = 0; // algae position
  //   if ( Reef_Position == 4)
  //       RobotOffset = RobotOffset +1; // move 1 meter further if removeing algae

        
  //   double x = (ReefRadius + RobotOffset) * Math.cos(Units.degreesToRadians((ReefSegment *60)+offsetAng)) + CenterofReefX;
  //   double y = (ReefRadius + RobotOffset) * Math.sin(Units.degreesToRadians((ReefSegment *60)+offsetAng)) + CenterofReefY;
  //   double rot = ReefSegment * 60;
  //   ReefPose = new Pose2d(x,y, Rotation2d.fromDegrees(rot));
    DogLog.log("Auto/DriveToReefV2Cmd/CommandStatus", "initialized");
    DogLog.log("Auto/DriveToReefV2Cmd/Init/Branch", branch);
    DogLog.log("Auto/DriveToReefV2Cmd/Init/targetPose", targetPose);

    XPID.setTolerance(0.025);
    YPID.setTolerance(0.025);
    RotPID.setTolerance(0.75);
    RotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DogLog.log("Auto/DriveToReefV2Cmd/CommandStatus", "executing");
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/CurrentRobotPose", DriveSS.getPose());

    double CurrentX = DriveSS.getPose().getX();
    double CurrentY = DriveSS.getPose().getY();
    double CurrentRot = DriveSS.getPose().getRotation().getDegrees();
  //   double Reef_X = ReefPose.getX();
  //   double Reef_Y = ReefPose.getY();
  //   double Reef_Rot = ReefPose.getRotation().getDegrees()+180;
    double XVel = 0;  
    double YVel =0;
  //   // if (RotPID.atSetpoint()){
      
      XVel = XPID.calculate(CurrentX, targetPose.getX());
      YVel = YPID.calculate(CurrentY, targetPose.getY());

  //   // }
    double RotVel = RotPID.calculate(CurrentRot,targetPose.getRotation().getDegrees() + 180);
    XVel = MathUtil.clamp(XVel, -3, 3);
    YVel = MathUtil.clamp(YVel, -3,3);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedXPIDOutput", XVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedYPIDOutput", YVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedRotPIDOutput", RotVel);

    if (!XPID.atSetpoint()){
      if (XVel > 0 ){
        XVel = Math.max(XVel, minSpeed);
      }
      else if (XVel < 0 ){
        XVel = Math.min(XVel, -minSpeed);
      }
    }
    if (XPID.atSetpoint()){
      XVel = XPID.calculate(CurrentX, targetPose.getX());
    }
    if (YPID.atSetpoint()){
      YVel = YPID.calculate(CurrentY, targetPose.getY());
    }
    if (!YPID.atSetpoint()){
      if (  YVel > 0 ){
        YVel = Math.max(YVel, minSpeed);
      }
      else if (YVel < 0 ){
        YVel = Math.min(YVel, -minSpeed);
      }
    }

    if(Math.abs(RotPID.getError()) >= 3.0){
      XVel = 0.1*XVel;
      YVel = 0.1*YVel;
      // System.out.println("Limpin'");
      // System.out.println(Math.abs(CurrentRot - (targetPose.getRotation().getDegrees() + 180)));
    
    }

    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);

    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualXOutput", XVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualYOutput", YVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualRotOutput", RotVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/XPIDError", XPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/YPIDError", YPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/RotPIDError", RotPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Check/XPIDAtSetpoint", XPID.atSetpoint());
    DogLog.log("Auto/DriveToReefV2Cmd/Check/YPIDAtSetpoint", YPID.atSetpoint());
    DogLog.log("Auto/DriveToReefV2Cmd/Check/RotPIDAtSetpoint", RotPID.atSetpoint());


    
   
    //System.out.println("Driving toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("Auto/DriveToReefV2Cmd/CommandStatus", "finished");
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    //System.out.println("DrivetoReef Finished");
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XPID.atSetpoint() && YPID.atSetpoint() && RotPID.atSetpoint();
  }

  public boolean wasInterrupted(){
    return inter;
  }
}
