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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPosition extends Command {

  Integer Reef_Side;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;

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

    double CenterofReefX  = 4.5; //wall to wall
    double CenterofReefY = 4; //aliance Wall to Alliance Wall
    double ReefRadius = Units.inchesToMeters(65.5)/2;
    //onshape cordinates of blue 
    //y along alliance wall
    //x from blue to red
    double LeftOffsetAng=-12;
    double RightOFfsetAng=12;
    double RobotOffset = Units.inchesToMeters(15); 
    
    double offsetAng = 0;
    System.out.println("Initializing Drive to Reef *****************");
    int ReefSegment = DriveSS.getReefSegment();
    System.out.println("ReefSegment" + ReefSegment);
    if (Reef_Side ==1)
      offsetAng = LeftOffsetAng;
    if (Reef_Side == 2)
      offsetAng = RightOFfsetAng;

    double x = (ReefRadius + RobotOffset) * Math.cos(Units.degreesToRadians(ReefSegment *60)+offsetAng) + CenterofReefX;
    double y = (ReefRadius + RobotOffset) * Math.sin(Units.degreesToRadians(ReefSegment *60)+offsetAng) + CenterofReefY;
    double rot = ReefSegment * 60;
    ReefPose = new Pose2d(x,y, Rotation2d.fromDegrees(rot));
    DogLog.log ("ReefScore/ReefPose", ReefPose);


    
    
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
    double Reef_X = ReefPose.getX();
    double Reef_Y = ReefPose.getY();
    double Reef_Rot = ReefPose.getRotation().getDegrees();

    double XVel = XPID.calculate(CurrentX, Reef_X);
    double YVel = YPID.calculate(CurrentY, Reef_Y);
    double RotVel = RotPID.calculate(CurrentRot,Reef_Rot);
    XVel = MathUtil.clamp(XVel, -0.2, 0.2);
    YVel = MathUtil.clamp(YVel, -0.2,0.2);
    RotVel = MathUtil.clamp(RotVel, -0.2, 0.2);
    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);
    DogLog.log("ReefScore/Xvel",XVel);
    DogLog.log("ReefScore/Yvel",YVel);
    DogLog.log("ReefScore/Rotvel",RotVel);
   
    System.out.println("Driving toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    System.out.println("DrivetoReef Finished");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XPID.atSetpoint() & YPID.atSetpoint() & RotPID.atSetpoint();
  }
}
