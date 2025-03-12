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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToFeederPosition extends Command {
  /** Creates a new CmdT_DriveToFeederPosition. */

  SwerveSubsystem DriveSS;
  Pose2d FeederPose;
  double Feeder_X;
  double Feeder_Y;
  double Feeder_Rot;
  PIDController XPID = new PIDController(2, 0, 0);
  PIDController YPID = new PIDController(2, 0, 0);
  PIDController RotPID = new PIDController(0.1,0,0);
  boolean StartingPositionOK = false;
  public CmdT_DriveToFeederPosition(SwerveSubsystem DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d CurrentLocation = DriveSS.getPose();
    Pose2d TargetPose = new Pose2d (0,0, Rotation2d.fromDegrees(0));
    DriveSS.DH_Out_AtCoralStation = false;
    double FieldWidth = 8; //wall to wall
    double FieldDepth = 17.55; //aliance Wall to Alliance Wall
    //onshape cordinates of blue 
    //y along alliance wall
    //x from blue to red
    //double s1x=8.435, s1y=3.012;
    // double s2x=8.273, s2y=3.134;
    // double s3x=8.107, s3y=3.25;
    // double s4x=7.942, s4y=3.37;
    // double s5x=7.778, s5y=3.489;
    // double s6x=7.613 , s6y=3.609;
    // double s7x=7.449, s7y=3.728;

    double s1x=Units.inchesToMeters(13.5), s1y=Units.inchesToMeters(38.25);
    double s2x=Units.inchesToMeters(20.25), s2y=Units.inchesToMeters(33.5);
    double s3x=Units.inchesToMeters(26.5), s3y=Units.inchesToMeters(29);
    double s4x=Units.inchesToMeters(33.5), s4y=Units.inchesToMeters(24.5);
    double s5x=Units.inchesToMeters(39.75), s5y=Units.inchesToMeters(19.5);
    double s6x=Units.inchesToMeters(46), s6y=Units.inchesToMeters(14.75);
    double s7x=Units.inchesToMeters(52.5), s7y=Units.inchesToMeters(10);
  
    double TargetRot=0;
    double RobotOffsetX = Units.inchesToMeters(8); //38.6cm
    double RobotOffsetY = Units.inchesToMeters(10.5);
    System.out.println("Initializing Drive to Feeder *****************");
    System.out.println("blue Alliance"+ DriveSS.DH_OUT_isBlueAlliance);

    if (CurrentLocation.getX() < 2.5 && CurrentLocation.getY() < 2.5 && DriveSS.DH_OUT_isBlueAlliance)
    {
      //At Tag 12 - Blue Alliance 
      //onshape cordinates -> World Co-ordinates (8.75-x, 4-y) -> Robot Cordinates (Wx+.386, Wy +.322)
      
      //slot 1 -8.41, -3.03 -> 0.34, 0.97
      StartingPositionOK = true;
      s1x = s1x + RobotOffsetX;
      s1y =  s1y + RobotOffsetY;
      //slot 2 -8.245, -3.15 -> 0.505, 0.85
      s2x = s2x + RobotOffsetX;
      s2y =  s2y + RobotOffsetY;
            //slot 3 -8.081, -3.269 -> 0.669, 0.731
      s3x =  s3x + RobotOffsetX;
      s3y =  s3y + RobotOffsetY;
      //Slot 4 -7.916, -3.388 -> 0.834, 0.612
      s4x = s4x + RobotOffsetX;
      s4y =  s4y + RobotOffsetY;
      //slot 5 -7.752, -3,508 -> 0.998, 0.492
      s5x = s5x + RobotOffsetX;
      s5y =  s5y + RobotOffsetY;
      //slot 6 -7.588, -3.627 -> 1.162, 0.373
      s6x = s6x + RobotOffsetX;
      s6y =  s6y + RobotOffsetY;
      //slot 7 -7.423, -3.747 -> 1.327, 0.253
      s7x =  s7x + RobotOffsetX;
      s7y =  s7y + RobotOffsetY;
      // TargetRot = -126;
      TargetRot = 54;
      //middle slot is at 0.834m,0.612m (1.22, .93) (Add .386, .322 to Slot Locations)

    }
    if (CurrentLocation.getX() < 2.5 && CurrentLocation.getY() > 5.5 && DriveSS.DH_OUT_isBlueAlliance)
    {
      double OffsetX = 0;
      double OffsetY = 8;
      StartingPositionOK = true;
      s1x =  s1x + RobotOffsetX;
      s1y =  OffsetY - s1y - RobotOffsetY;
      //slot 2 -8.245, -3.15 -> 0.505, 0.85
      s2x = s2x - RobotOffsetX;
      s2y =  OffsetY - s2y - RobotOffsetY;
            //slot 3 -8.081, -3.269 -> 0.669, 0.731
      s3x =  s3x - RobotOffsetX;
      s3y =  OffsetY - s3y - RobotOffsetY;
      //Slot 4 -7.916, -3.388 -> 0.834, 0.612
      s4x = s4x - RobotOffsetX;
      s4y =  OffsetY - s4y - RobotOffsetY;
      //slot 5 -7.752, -3,508 -> 0.998, 0.492
      s5x = s5x - RobotOffsetX;
      s5y =  OffsetY - s5y - RobotOffsetY;
      //slot 6 -7.588, -3.627 -> 1.162, 0.373
      s6x = s6x - RobotOffsetX;
      s6y =  OffsetY - s6y - RobotOffsetY;
      //slot 7 -7.423, -3.747 -> 1.327, 0.253
      s7x =   s7x - RobotOffsetX;
      s7y =  OffsetY - s7y - RobotOffsetY;
      // TargetRot = -126;
      TargetRot = -54;
      //At Tag 13 - Blue Alliance
    }
    if (CurrentLocation.getX() > 15 && CurrentLocation.getY() < 2.5 && DriveSS.DH_OUT_isRedAlliance)
    {
      //At Tag 1 - Red Alliance
      double OffsetX = 17.5;
      double OffsetY = 0;
      StartingPositionOK = true;
      s1x = OffsetX - s1x - RobotOffsetX;
      s1y = OffsetY - s1y + RobotOffsetY;
      //slot 2 -8.245, -3.15 -> 0.505, 0.85
      s2x = OffsetX - s2x - RobotOffsetX;
      s2y = OffsetY - s2y + RobotOffsetY;
            //slot 3 -8.081, -3.269 -> 0.669, 0.731
      s3x =  OffsetX - s3x - RobotOffsetX;
      s3y =  OffsetY - s3y + RobotOffsetY;
      //Slot 4 -7.916, -3.388 -> 0.834, 0.612
      s4x = OffsetX -s4x - RobotOffsetX;
      s4y =  OffsetY - s4y + RobotOffsetY;
      //slot 5 -7.752, -3,508 -> 0.998, 0.492
      s5x = OffsetX -s5x - RobotOffsetX;
      s5y =  OffsetY - s5y + RobotOffsetY;
      //slot 6 -7.588, -3.627 -> 1.162, 0.373
      s6x = OffsetX -s6x - RobotOffsetX;
      s6y =  OffsetY - s6y + RobotOffsetY;
      //slot 7 -7.423, -3.747 -> 1.327, 0.253
      s7x =  OffsetX - s7x - RobotOffsetX;
      s7y =  OffsetY - s7y + RobotOffsetY;
      // TargetRot = -126;
      TargetRot = 54+90;
      
    }
    if (CurrentLocation.getX() >15 && CurrentLocation.getY() > 5.5 && DriveSS.DH_OUT_isRedAlliance)
    {
     //At Tag 1 - Red Alliance
     double OffsetX = 17.5;
     double OffsetY = 8;
     StartingPositionOK = true;
     s1x = OffsetX - s1x - RobotOffsetX;
     s1y = OffsetY - s1y + RobotOffsetY;
     //slot 2 -8.245, -3.15 -> 0.505, 0.85
     s2x = OffsetX - s2x - RobotOffsetX;
     s2y = OffsetY - s2y + RobotOffsetY;
           //slot 3 -8.081, -3.269 -> 0.669, 0.731
     s3x =  OffsetX - s3x - RobotOffsetX;
     s3y =  OffsetY - s3y + RobotOffsetY;
     //Slot 4 -7.916, -3.388 -> 0.834, 0.612
     s4x = OffsetX -s4x - RobotOffsetX;
     s4y =  OffsetY - s4y + RobotOffsetY;
     //slot 5 -7.752, -3,508 -> 0.998, 0.492
     s5x = OffsetX -s5x - RobotOffsetX;
     s5y =  OffsetY - s5y + RobotOffsetY;
     //slot 6 -7.588, -3.627 -> 1.162, 0.373
     s6x = OffsetX -s6x - RobotOffsetX;
     s6y =  OffsetY - s6y + RobotOffsetY;
     //slot 7 -7.423, -3.747 -> 1.327, 0.253
     s7x =  OffsetX - s7x - RobotOffsetX;
     s7y =  OffsetY - s7y + RobotOffsetY;
     // TargetRot = -126;
     TargetRot = 54+90; //At Tag 2 - Red Alliance
    }

    double CurrentX = CurrentLocation.getX();
    double CurrentY = CurrentLocation.getY();
    double DistanceS1 = Math.sqrt(Math.pow((CurrentX - s1x),2)+Math.pow((CurrentY - s1y),2));
    double DistanceS2 = Math.sqrt(Math.pow((CurrentX - s2x),2)+Math.pow((CurrentY - s2y),2));
    double DistanceS3 = Math.sqrt(Math.pow((CurrentX - s3x),2)+Math.pow((CurrentY - s3y),2));
    double DistanceS4 = Math.sqrt(Math.pow((CurrentX - s4x),2)+Math.pow((CurrentY - s4y),2));
    double DistanceS5 = Math.sqrt(Math.pow((CurrentX - s5x),2)+Math.pow((CurrentY - s5y),2));
    double DistanceS6 = Math.sqrt(Math.pow((CurrentX - s6x),2)+Math.pow((CurrentY - s6y),2));
    double DistanceS7 = Math.sqrt(Math.pow((CurrentX - s7x),2)+Math.pow((CurrentY - s7y),2));

  Translation2d CurrentTranslation = CurrentLocation.getTranslation();
  Translation2d S1 = new Translation2d(s1x,s1y);
  Translation2d S2 = new Translation2d(s2x,s2y);
  Translation2d S3 = new Translation2d(s3x,s3y);
  Translation2d S4 = new Translation2d(s4x,s4y);
  Translation2d S5 = new Translation2d(s5x,s5y);
  Translation2d S6 = new Translation2d(s6x,s6y);
  Translation2d S7 = new Translation2d(s7x,s7y);
  
    double closest = Math.min(DistanceS1, DistanceS2);
    closest = Math.min(closest, DistanceS3);
    closest = Math.min(closest, DistanceS4);
    closest = Math.min(closest, DistanceS5);
    closest = Math.min(closest, DistanceS6);
    closest = Math.min(closest, DistanceS7);
    System.out.println("closest " + closest );

    if (DistanceS1 == closest)
    {
      TargetPose = new Pose2d(S1,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 1);
    }
    if (DistanceS2 == closest)
    {
      TargetPose = new Pose2d(S2,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 2);
    }
    if (DistanceS3 == closest)
    {
      TargetPose = new Pose2d(S3,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 3);
    }
    if (DistanceS4 == closest)
    {
      TargetPose = new Pose2d(S4,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 4);
    }
    if (DistanceS5 == closest)
    {
      TargetPose = new Pose2d(S5,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 5);
    }
    if (DistanceS6 == closest)
    {
      TargetPose = new Pose2d(S6,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 6);
    }
    if (DistanceS7 == closest)
    {
      TargetPose = new Pose2d(S7,Rotation2d.fromDegrees(TargetRot));
      DogLog.log("Target slot", 7);
    }
    Feeder_X = TargetPose.getX();
    Feeder_Y = TargetPose.getY();
    Feeder_Rot = TargetPose.getRotation().getDegrees();
    FeederPose = new Pose2d(Feeder_X, Feeder_Y,Rotation2d.fromDegrees(Feeder_Rot));
    XPID.setTolerance(0.05);
    YPID.setTolerance(0.05);
    RotPID.setTolerance(5);
    RotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CurrentX = DriveSS.getPose().getX();
    double CurrentY = DriveSS.getPose().getY();
    double CurrentRot = DriveSS.getPose().getRotation().getDegrees();
    double RotVel = RotPID.calculate(CurrentRot,Feeder_Rot);
    double XVel = 0;  
    double YVel =0;
    if (RotPID.atSetpoint()){
     XVel = XPID.calculate(CurrentX, Feeder_X);
     YVel = YPID.calculate(CurrentY, Feeder_Y);
    }
    
    XVel = MathUtil.clamp(XVel, -2, 2);
    YVel = MathUtil.clamp(YVel, -2, 2);
    RotVel = MathUtil.clamp(RotVel, -1, 1);
    if (StartingPositionOK)
    {
      DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);
    }
    DogLog.log("Xvel",XVel);
    DogLog.log("Yvel",YVel);
    DogLog.log("Rotvel",RotVel);
    DogLog.log("FeederPose", FeederPose);
    //System.out.println("Driving toFeeder");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    System.out.println("DrivetoFeeder Finished");
    System.out.println ("XPID" + XPID.atSetpoint());
    System.out.println ("YPID" + YPID.atSetpoint());
    System.out.println ("RotPID" + RotPID.atSetpoint());
    
    if (XPID.atSetpoint() & YPID.atSetpoint() & RotPID.atSetpoint())
    {
      
      System.out.println("At CoralStation");
        DriveSS.DH_Out_AtCoralStation = true;
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (XPID.atSetpoint() & YPID.atSetpoint() & RotPID.atSetpoint()) || !StartingPositionOK;
  } 
}
