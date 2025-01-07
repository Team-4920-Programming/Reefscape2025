// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BoltLog;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import java.lang.Math;
import frc.robot.BoltLog;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public Pose2d robotSimulationWorldPose;
  public ChassisSpeeds chassisSpeedsFieldRelative;
  public boolean RobotHasGP = false;
  public boolean ShotComplete = false;
  private Pose3d TargetPose3d;
  private Pose2d RobotPose2d;
  private boolean AimEnabled = false;
  private double AimAngle = 15;
  public ShooterSubsystem() {}
  private final BoltLog BoltLogger = new BoltLog();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotHasGP)
      ShotComplete = false;

    if (AimEnabled & RobotHasGP)
      UpdateAim();

    BoltLogger.Log(BoltLogger.HighLog,"ShooterSubSystem","None","Periodic","AimAngle",AimAngle);
    BoltLogger.Log(BoltLogger.HighLog,"ShooterSubSystem","None","Periodic","AimEnabled",AimEnabled);
    BoltLogger.Log(BoltLogger.HighLog,"ShooterSubSystem","None","Periodic","RobotHasGP",RobotHasGP);
    BoltLogger.Log(BoltLogger.HighLog,"ShooterSubSystem","None","Periodic","Target",TargetPose3d);
    
    if (Robot.isSimulation())
    {
      
    }


  }
  
  public void AimAtTarget (Pose3d TargetPose, Pose2d RobotPose){
    TargetPose3d = TargetPose;
    AimEnabled = true;

  }
  private void UpdateAim(){
    double xdiff = TargetPose3d.getX() - RobotPose2d.getX();
    double ydiff = TargetPose3d.getY() - RobotPose2d.getY();
    double Ldist = Math.sqrt (xdiff*xdiff + ydiff*ydiff);
    double zdiff = TargetPose3d.getZ();
    double AimAng = Math.toDegrees(Math.atan2(zdiff,Ldist));
    AimAngle = AimAng;




  }
  public void ShootSimGP(){
    if (RobotHasGP)
    {
    double velocityRPM = 5400;
    NoteOnFly noteOnFly = new NoteOnFly(
        // Specify the position of the chassis when the note is launched
        robotSimulationWorldPose.getTranslation(),
        // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
        new Translation2d(0.2, 0),
        // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
        chassisSpeedsFieldRelative,
        // The shooter facing direction is the same as the robot’s facing direction
        robotSimulationWorldPose.getRotation(),
                // Add the shooter’s rotation
               
        // Initial height of the flying note
        0.45,
        // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
        velocityRPM / 6000 * 20,
        // The angle at which the note is launched
        Math.toRadians(AimAngle)
    );
    noteOnFly.addGamePieceAfterTouchGround(SimulatedArena.getInstance());
    noteOnFly.enableBecomeNoteOnFieldAfterTouchGround();
    noteOnFly.asSpeakerShotNote(null);
    SimulatedArena.getInstance().addGamePieceProjectile(noteOnFly);
    ShotComplete = true;
    RobotHasGP = false;
    }
  }
}
