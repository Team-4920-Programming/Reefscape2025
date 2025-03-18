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
  PIDController XPID = new PIDController(2, 0, 0.02);
  PIDController YPID = new PIDController(2, 0, 0.02);
  PIDController RotPID = new PIDController(0.15,0,0);
  double minSpeed = 0.2;
  boolean inter;
  Twist2d test;
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

    startingPose = DriveSS.getPose();

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

    Pose2d currentPose = DriveSS.getPose();

    test = currentPose.log(targetPose);

    Pose2d test3 = currentPose.rotateAround(currentPose.getTranslation(), new Rotation2d(test.dtheta));
    Transform2d test2 = targetPose.minus(currentPose);
  //   // if (RotPID.atSetpoint()){

    DogLog.log("Auto/DriveToReefV2Cmd/Exec/TwistDx", test.dx);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/TwistDy", test.dy);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/TwistDtheta", Units.radiansToDegrees(test.dtheta));
   
      DogLog.log("Auto/DriveToReefV2Cmd/Exec/test3Pose", test3);
      double XVel = 0;
      double YVel = 0;
      XVel = test.dx * 2;
      YVel = test.dy * 2;
      // XVel = XPID.calculate(CurrentX, targetPose.getX());
      // YVel = YPID.calculate(CurrentY, targetPose.getY());

  //   // }
    // double RotVel = RotPID.calculate(DriveSS.getPose().getRotation().getDegrees(),targetPose.getRotation().getDegrees());
    double RotVel = test.dtheta * 5;
    XVel = MathUtil.clamp(XVel, -3, 3);
    YVel = MathUtil.clamp(YVel, -3,3);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedXPIDOutput", XVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedYPIDOutput", YVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ClampedRotPIDOutput", RotVel);

    if (!atSetpoint(XVel, 0.03)){
      if (XVel > 0 ){
        XVel = Math.max(XVel, minSpeed);
      }
      else if (XVel < 0 ){
        XVel = Math.min(XVel, -minSpeed);
      }
    }
    if (atSetpoint(XVel, 0.03)){
      XVel = XPID.calculate(CurrentX, targetPose.getX());
    }
    if (atSetpoint(YVel, 0.03)){
      YVel = YPID.calculate(CurrentY, targetPose.getY());
    }
    if (!atSetpoint(YVel, 0.03)){
      if (  YVel > 0 ){
        YVel = Math.max(YVel, minSpeed);
      }
      else if (YVel < 0 ){
        YVel = Math.min(YVel, -minSpeed);
      }
    }

    // if(Math.abs(RotPID.getError()) >= 3.0){
    //   XVel = 0.1*XVel;
    //   YVel = 0.1*YVel;
      // System.out.println("Limpin'");
      // System.out.println(Math.abs(CurrentRot - (targetPose.getRotation().getDegrees() + 180)));
    
    // }  
    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,false);

    // DriveSS.drive(new Translation2d(test.dx*3,test.dy*3),test.dtheta*3,false);

    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualXOutput", XVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualYOutput", YVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/ActualRotOutput", RotVel);
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/XPIDError", XPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/YPIDError", YPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Exec/RotPIDError", RotPID.getError());
    DogLog.log("Auto/DriveToReefV2Cmd/Check/XPIDAtSetpoint", Math.abs(test.dx) < 0.03);
    DogLog.log("Auto/DriveToReefV2Cmd/Check/YPIDAtSetpoint", Math.abs(test.dy) < 0.03);
    DogLog.log("Auto/DriveToReefV2Cmd/Check/RotPIDAtSetpoint", Math.abs(test.dtheta) < Units.degreesToRadians(1));


    
   
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
    DogLog.log("Auto/DriveToReefV2Cmd/End/Interrupted", interrupted); 
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(test.dx) <= 0.03 && Math.abs(test.dy) <= 0.03 && RotPID.atSetpoint() && Math.abs((test.dtheta)) <= Units.degreesToRadians(1);
  }

  public boolean wasInterrupted(){
    return inter;
  }

  public boolean atSetpoint(double error, double tolerance){
    return Math.abs(error) <= tolerance;
  }
}
