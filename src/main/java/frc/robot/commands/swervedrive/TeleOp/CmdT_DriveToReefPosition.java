// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPosition extends Command {

  Integer Reef_Side;
 
  SwerveSubsystem DriveSS;
  Pose2d FeederPose;
  double Feeder_X;
  double Feeder_Y;
  double Feeder_Rot;
  PIDController XPID = new PIDController(0.5, 0, 0);
  PIDController YPID = new PIDController(0.5, 0, 0);
  PIDController RotPID = new PIDController(0.1,0,0);
  public CmdT_DriveToReefPosition(SwerveSubsystem DriveSubsystem, Integer Side) {
    // Use addRequirements() here to declare subsystem dependencies.
    Reef_Side = Side;
      DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d CurrentLocation = DriveSS.getPose();
    Pose2d TargetPose = new Pose2d (0,0, Rotation2d.fromDegrees(0));
    double CenterofReefX  = 4.5; //wall to wall
    double CenterofReefY = 4; //aliance Wall to Alliance Wall
    //onshape cordinates of blue 
    //y along alliance wall
    //x from blue to red
    double s1x=8.435, s1y=3.012;
    double s2x=8.273, s2y=3.134;
    double s3x=8.107, s3y=3.25;
    double s4x=7.942, s4y=3.37;
    double s5x=7.778, s5y=3.489;
    double s6x=7.613 , s6y=3.609;
    double s7x=7.449, s7y=3.728;
    double TargetRot=0;
    double RobotOffsetX = 0.386; //38.6cm
    double RobotOffsetY = 0.322; //32.2cm
    System.out.println("Initializing Drive to Feeder *****************");
    if (CurrentLocation.getX() < 2.5 && CurrentLocation.getY() < 2.5)
    {
      //At Tag 12 - Blue Alliance 
      //onshape cordinates -> World Co-ordinates (8.75-x, 4-y) -> Robot Cordinates (Wx+.386, Wy +.322)
      
      //slot 1 -8.41, -3.03 -> 0.34, 0.97
      s1x = CenterofReefX/2 - s1x + RobotOffsetX;
      s1y = CenterofReefY/2 - s1y + RobotOffsetY;
      //slot 2 -8.245, -3.15 -> 0.505, 0.85
      s2x = CenterofReefX/2 - s2x + RobotOffsetX;
      s2y = CenterofReefY/2 - s2y + RobotOffsetY;
            //slot 3 -8.081, -3.269 -> 0.669, 0.731
      s3x = CenterofReefX/2 - s3x + RobotOffsetX;
      s3y = CenterofReefY/2 - s3y + RobotOffsetY;
      //Slot 4 -7.916, -3.388 -> 0.834, 0.612
      s4x = CenterofReefX/2 - s4x + RobotOffsetX;
      s4y = CenterofReefY/2 - s4y + RobotOffsetY;
      //slot 5 -7.752, -3,508 -> 0.998, 0.492
      s5x = CenterofReefX/2 - s5x + RobotOffsetX;
      s5y = CenterofReefY/2 - s5y + RobotOffsetY;
      //slot 6 -7.588, -3.627 -> 1.162, 0.373
      s6x = CenterofReefX/2 - s6x + RobotOffsetX;
      s6y = CenterofReefY/2 - s6y + RobotOffsetY;
      //slot 7 -7.423, -3.747 -> 1.327, 0.253
      s7x = CenterofReefX/2 - s7x + RobotOffsetX;
      s7y = CenterofReefY/2 - s7y + RobotOffsetY;
      
      System.out.println("s1x:"+s1x+" s1y:"+s1y);
      System.out.println("s2x:"+s2x+" s2y:"+s2y);
      System.out.println("s3x:"+s3x+" s3y:"+s3y);
      System.out.println("s4x:"+s4x+" s4y:"+s4y);
      System.out.println("s5x:"+s5x+" s5y:"+s5y);
      System.out.println("s6x:"+s6x+" s6y:"+s6y);
      System.out.println("s7x:"+s7x+" s7y:"+s7y);
      System.out.println("Cx:"+CurrentLocation.getX()+" Cy:"+CurrentLocation.getY());


      TargetRot = -126;
      //middle slot is at 0.834m,0.612m (1.22, .93) (Add .386, .322 to Slot Locations)

    }
    if (CurrentLocation.getX() < 2.5 && CurrentLocation.getY() > 5.5)
    {
      //At Tag 13 - Blue Alliance
    }
    if (CurrentLocation.getX() > 15 && CurrentLocation.getY() < 2.5)
    {
      //At Tag 1 - Red Alliance
    }
    if (CurrentLocation.getX() >15 && CurrentLocation.getY() > 5.5)
    {
      //At Tag 2 - Red Alliance
    }

    double CurrentX = CurrentLocation.getX();
    double CurrentY = CurrentLocation.getY();
    double DistanceS1 = Math.sqrt(Math.pow((CurrentX - s1x),2)/Math.pow((CurrentY - s1y),2));
    double DistanceS2 = Math.sqrt(Math.pow((CurrentX - s2x),2)/Math.pow((CurrentY - s2y),2));
    double DistanceS3 = Math.sqrt(Math.pow((CurrentX - s3x),2)/Math.pow((CurrentY - s3y),2));
    double DistanceS4 = Math.sqrt(Math.pow((CurrentX - s4x),2)/Math.pow((CurrentY - s4y),2));
    double DistanceS5 = Math.sqrt(Math.pow((CurrentX - s5x),2)/Math.pow((CurrentY - s5y),2));
    double DistanceS6 = Math.sqrt(Math.pow((CurrentX - s6x),2)/Math.pow((CurrentY - s6y),2));
    double DistanceS7 = Math.sqrt(Math.pow((CurrentX - s7x),2)/Math.pow((CurrentY - s7y),2));

  Translation2d CurrentTranslation = CurrentLocation.getTranslation();
  Translation2d S1 = new Translation2d(s1x,s1y);
  Translation2d S2 = new Translation2d(s2x,s2y);
  Translation2d S3 = new Translation2d(s3x,s3y);
  Translation2d S4 = new Translation2d(s4x,s4y);
  Translation2d S5 = new Translation2d(s5x,s5y);
  Translation2d S6 = new Translation2d(s6x,s6y);
  Translation2d S7 = new Translation2d(s7x,s7y);
  
    // double DistanceS1 = CurrentTranslation.minus(S1).getNorm();
    System.out.println("\t\"Distance\": {\"S1\": " + DistanceS1 + " },");
    // double DistanceS2 = CurrentTranslation.minus(S2).getNorm();
    System.out.println("\t\"Distance\": {\"S2\": " + DistanceS2 + " },");
    // double DistanceS3 = CurrentTranslation.minus(S3).getNorm();
    System.out.println("\t\"Distance\": {\"S3\": " + DistanceS3 + " },");
    // double DistanceS4 = CurrentTranslation.minus(S4).getNorm();
    System.out.println("\t\"Distance\": {\"S4\": " + DistanceS4 + " },");
    // double DistanceS5 = CurrentTranslation.minus(S5).getNorm();
    System.out.println("\t\"Distance\": {\"S5\": " + DistanceS5 + " },");
    // double DistanceS6 = CurrentTranslation.minus(S6).getNorm();
    System.out.println("\t\"Distance\": {\"S6\": " + DistanceS6 + " },");
    // double DistanceS7 = CurrentTranslation.minus(S7).getNorm();
    System.out.println("\t\"Distance\": {\"S7\": " + DistanceS7 + " },");

    double closest = Math.min(DistanceS1, DistanceS2);
    closest = Math.min(closest, DistanceS3);
    closest = Math.min(closest, DistanceS4);
    closest = Math.min(closest, DistanceS5);
    closest = Math.min(closest, DistanceS6);
    closest = Math.min(closest, DistanceS7);
   
    if (DistanceS1 == closest)
    {
      TargetPose = new Pose2d(S1,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 1);
    }
    if (DistanceS2 == closest)
    {
      TargetPose = new Pose2d(S2,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 2);
    }
    if (DistanceS3 == closest)
    {
      TargetPose = new Pose2d(S3,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 3);
    }
    if (DistanceS4 == closest)
    {
      TargetPose = new Pose2d(S4,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 4);
    }
    if (DistanceS5 == closest)
    {
      TargetPose = new Pose2d(S5,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 5);
    }
    if (DistanceS6 == closest)
    {
      TargetPose = new Pose2d(S6,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 6);
    }
    if (DistanceS7 == closest)
    {
      TargetPose = new Pose2d(S7,Rotation2d.fromDegrees(-126));
      DogLog.log("Target slot", 7);
    }
    Feeder_X = TargetPose.getX();
    Feeder_Y = TargetPose.getY();
    Feeder_Rot = TargetPose.getRotation().getDegrees();
    FeederPose = new Pose2d(Feeder_X, Feeder_Y,Rotation2d.fromDegrees(Feeder_Rot));
    XPID.setTolerance(0.1);
    YPID.setTolerance(0.1);
    RotPID.setTolerance(5);
    RotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CurrentX = DriveSS.getPose().getX();
    double CurrentY = DriveSS.getPose().getY();
    double CurrentRot = DriveSS.getPose().getRotation().getDegrees();
    double XVel = XPID.calculate(CurrentX, Feeder_X);
    double YVel = YPID.calculate(CurrentY, Feeder_Y);
    double RotVel = RotPID.calculate(CurrentRot,Feeder_Rot);
    XVel = MathUtil.clamp(XVel, -0.2, 0.2);
    YVel = MathUtil.clamp(YVel, -0.2,0.2);
    RotVel = MathUtil.clamp(RotVel, -0.2, 0.2);
    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);
    DogLog.log("Xvel",XVel);
    DogLog.log("Yvel",YVel);
    DogLog.log("Rotvel",RotVel);
    DogLog.log("FeederPose", FeederPose);
    System.out.println("Driving toFeeder");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    System.out.println("DrivetoFeeder Finished");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XPID.atSetpoint() & YPID.atSetpoint() & RotPID.atSetpoint();
  }
}
