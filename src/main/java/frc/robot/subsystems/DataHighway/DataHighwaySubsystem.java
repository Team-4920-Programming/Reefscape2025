// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.DataHighway;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Observable;

import dev.doglog.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
public class DataHighwaySubsystem extends SubsystemBase {
  /** Creates a new DataHighwaySubsystem. */
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;
  private boolean hasCoral = false;
  private double ReefDistance = 0;
  private int ReefSegment = 0;
  private boolean InRedZone = true;
  private boolean inReefRedZone = true;
  private boolean inStationRedZone = true;
  private boolean inCageRedZone = false;
  private boolean inCoralStationRedZone = true;
  private boolean inCoralStationPickupZone = false;
  private boolean inReefYellowZone = false;
  private boolean InReefYellowZone = false;
  private boolean InCoralSationZone = false;
  private boolean AtCoralStation = false;
  private boolean isRedAlliance = false;
  private Pose3d LeftCoralStationPose = new Pose3d();
  private Pose3d RightCoralStationPose = new Pose3d();
  private Pose2d ReefPose = new Pose2d();
  private SwerveSubsystem Drive_SS;
  private CoralElevatorSubsystem Coral_SS;
  private AprilTagFieldLayout atf;
  private boolean inLeftCoralZone = false;
  private boolean inRightCoralZone = false;
  private Pose2d CurrentPose = new Pose2d();
  private Pose2d ClosestReefSegment = new Pose2d();
  private Pose2d ClosestPickupSlot = new Pose2d();
  private boolean isMatchSetupCompleted = false;
  List<Pose2d> reefPoses = new ArrayList();
  List<Pose2d> feederStationPoses = new ArrayList<>();
  public DataHighwaySubsystem(SwerveSubsystem DriveSS, CoralElevatorSubsystem CoralSS) {
    Drive_SS = DriveSS;
    Coral_SS = CoralSS;
    m_ledBuffer = new AddressableLEDBuffer(142);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    atf = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    reefPoses.add(new Pose2d().kZero);
    feederStationPoses.add(new Pose2d().kZero);
  }

  @Override
  public void periodic() {
    // if(!isMatchSetupCompleted){
    //   if ((DriverStation.isFMSAttached() || Robot.isSimulation()) && DriverStation.isDSAttached()){
      //     isMatchSetupCompleted = true;
      // }
    // }
    // SetupZonesCenter();
    // SetupReefPoses();
    CheckZones();
    CalculateClosestReefSegment();
    if ((inLeftCoralZone || inRightCoralZone) && !hasCoral)
    {
      CalculateClosestPickupSlot();
    }

    if (Robot.isSimulation())
    {
      sim();
    }
    double ReefRadius = Units.inchesToMeters(76.5)/2;
    CurrentPose = Drive_SS.getPose();
    getDriveData();
    getCoralData();
    setDriveData();
    setCoralData();
    double distancefromReefWall = ReefDistance- ReefRadius;
    DogLog.log ("DistanceToReefWall", distancefromReefWall);
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern CoralRainbow = LEDPattern.rainbow(255,128);

    SmartDashboard.putBoolean("InLeftCoralZone", inLeftCoralZone);
    SmartDashboard.putBoolean("InRightCoralZone", inRightCoralZone);
    DogLog.log("Zones/LeftCoralPose",LeftCoralStationPose);
    DogLog.log("Zones/RightCoralPose", RightCoralStationPose);
    
    if (inReefRedZone || inCageRedZone || inCoralStationRedZone){
      red.applyTo(m_ledBuffer);
    }
    else if(inReefYellowZone){
      yellow.applyTo(m_ledBuffer);
    }
    else if(hasCoral)
    {
      white.applyTo(m_ledBuffer);
    }
    else if(!hasCoral && (inCoralStationPickupZone)){
      CoralRainbow.applyTo(m_ledBuffer);
    }
    else{
      green.applyTo(m_ledBuffer);
    }

    SmartDashboard.putBoolean("inReefRedZone", inReefRedZone);
    SmartDashboard.putBoolean("inLeftCoralZone", inLeftCoralZone);
    SmartDashboard.putBoolean("inRightCoralZone", inRightCoralZone);
    SmartDashboard.putBoolean("inCageRedZone", inCageRedZone);
    SmartDashboard.putBoolean("inCoralStationRedZone", inCoralStationRedZone);
    SmartDashboard.putBoolean("inReefYellowZone", inReefYellowZone);

  //   if (hasCoral){
  //     AtCoralStation = false;
  //     white.applyTo(m_ledBuffer);
  //     if (distancefromReefWall < 2 & distancefromReefWall > 0.85){
  //       yellow.applyTo(m_ledBuffer);
  //       InReefYellowZone = true;
  //     }
        

  //   }
  //   else
  //     green.applyTo(m_ledBuffer);
  //     if (distancefromReefWall < 2 & distancefromReefWall > 0.85){
  //       yellow.applyTo(m_ledBuffer);
  //       InReefYellowZone = true;
  //     }

  //   //red zone - reduced speed - no elevator functions
  //   InRedZone = false; 
  //   if (distancefromReefWall < 0.85 & distancefromReefWall > 0.0)
  //   {
      
  //     InRedZone = true;
  //   }
  //   if (CurrentPose.getX() > 8 && CurrentPose.getX() < 9.5)
  //   {
  //     //in climber zone
  //     InRedZone = true;
  //   }
    
  //   // TODO: redzone Coral Stations
  //   // 3.3m from 0 to 3.3m (on a 45)

  //   double CurX = CurrentPose.getX();
  //  double CurY = CurrentPose.getY();
  //  double CurR = CurrentPose.getRotation().getDegrees();
  //   //double BlueRightr = Math.sqrt(Math.pow(CurX,2)+Math.pow(CurY,2));
  //   //double BlueLeftr = Math.sqrt (Math.pow())

  //   InCoralSationZone = false;
  //   if (inRightCoralZone){
  //     double RightRot = RightCoralStationPose.toPose2d().getRotation().getDegrees();
  //     //Right blue tag has a 54 degree pose, we set our robot to be between +90 and -90 from that pose
  //     if (CurR > RightRot +90  || CurR < RightRot -90){
  //       InRedZone = true;
  //     }
     
  //     InCoralSationZone =true;
  //   }
  //   if (inLeftCoralZone){
  //     double LeftRot = LeftCoralStationPose.toPose2d().getRotation().getDegrees();
  //     //left blue tag has a -54 degree pose, we set our robot to be between +90 and -90 from that pose
  //     if (CurR > LeftRot +90  || CurR < LeftRot -90){
  //       InRedZone = true;
  //     }
     
  //     InCoralSationZone =true;
  //   }
    
  //   if (InRedZone)
  //     red.applyTo(m_ledBuffer);
      
  //   if (AtCoralStation && !hasCoral)
  //   {
  //     //Ask for Coral
  //     CoralRainbow.applyTo(m_ledBuffer);
  //   }
  //   // Write the data to the LED strip
    m_led.setData(m_ledBuffer);

  }
  private void getDriveData()
  {
    ReefDistance = Drive_SS.DH_Out_ReefDistance;
    ReefSegment = Drive_SS.DH_Out_ReefSegment;   
    AtCoralStation = Drive_SS.DH_Out_AtCoralStation; 
    isRedAlliance = Drive_SS.DH_OUT_isRedAlliance;
  }
  private void setDriveData(){
    Drive_SS.DH_In_HasCoral = hasCoral;
    // Drive_SS.DH_In_InRedZone = InRedZone;
    // Drive_SS.DH_In_CoralYellow = InReefYellowZone;
    // Drive_SS.DH_InStationZone = InCoralSationZone;
    Drive_SS.DH_In_InRedZone = inReefRedZone || inCoralStationRedZone || inCageRedZone;
    Drive_SS.DH_In_CoralYellow = inReefYellowZone;
    Drive_SS.DH_InStationZone = inLeftCoralZone || inRightCoralZone;
    Drive_SS.DH_In_InLeftCoralZone = inLeftCoralZone;
    Drive_SS.DH_In_InRightCoralZone = inRightCoralZone;
    Drive_SS.DH_In_LeftCoralPose = LeftCoralStationPose.toPose2d();
    Drive_SS.DH_In_RightCoralPose = RightCoralStationPose.toPose2d();
    Drive_SS.DH_In_ClosestReefSegment = ClosestReefSegment;
    Drive_SS.DH_In_ReefPose = ReefPose;
    
  }
  private void getCoralData(){
    hasCoral = Coral_SS.DH_Out_HasCoral;
  }
  public Pose2d getClosestReefSegment(){
    return ClosestReefSegment;
  }
  public Pose2d getClosestPickupSlot(){
    return ClosestPickupSlot;
  }
  private void setCoralData(){
    // Coral_SS.DH_In_CoralZone = InCoralSationZone;
    // Coral_SS.DH_In_DistanceFromReef = ReefDistance;
    // Coral_SS.DH_In_RedZone = InRedZone;
    // Coral_SS.DH_In_YellowZone = InReefYellowZone;
    Coral_SS.DH_In_CoralZone = inLeftCoralZone || inRightCoralZone;
    Coral_SS.DH_In_DistanceFromReef = ReefDistance;
    Coral_SS.DH_In_RedZone = inReefRedZone || inCoralStationRedZone || inCageRedZone;
    Coral_SS.DH_In_YellowZone = inReefYellowZone;
    
  }

  private void SetupZonesCenter(){

    if (isRedAlliance){
        Pose2d tmp1 = atf.getTagPose(10).get().toPose2d();
        Pose2d tmp2 = atf.getTagPose(7).get().toPose2d();;
        LeftCoralStationPose = atf.getTagPose(1).get();
        RightCoralStationPose = atf.getTagPose(2).get();
        ReefPose = new Pose2d((tmp1.getX() + tmp2.getX())/2,tmp1.getY(),new Rotation2d().kZero);
    }
    else{
        Pose2d tmp1 = atf.getTagPose(18).get().toPose2d();
        Pose2d tmp2 = atf.getTagPose(21).get().toPose2d();;
        LeftCoralStationPose = atf.getTagPose(13).get();
        RightCoralStationPose = atf.getTagPose(12).get();
        ReefPose = new Pose2d((tmp1.getX() + tmp2.getX())/2,tmp1.getY(),new Rotation2d().kZero);
    }
    DogLog.log("Zones/ReefCenterPose", ReefPose);
  }
  public void StartUp(){
    feederStationPoses.clear();
    reefPoses.clear();
    SetupZonesCenter();
    SetupReefPoses();
    SetupCoralFeedPoses();
    isMatchSetupCompleted = true;

  }
  public boolean IsSetupCompleted(){
    return isMatchSetupCompleted;
  }

  public void ResetSetup(){
    isMatchSetupCompleted = false;
  }

  private void SetupReefPoses(){
    
    Pose2d pos1;
    Pose2d pos2;
    Pose2d pos3;
    Pose2d pos4;
    Pose2d pos5;
    Pose2d pos6;
    if (isRedAlliance){
    pos1 = atf.getTagPose(7).get().toPose2d();
    pos2 = atf.getTagPose(6).get().toPose2d();
    pos3 = atf.getTagPose(11).get().toPose2d();
    pos4 = atf.getTagPose(10).get().toPose2d();
    pos5 = atf.getTagPose(9).get().toPose2d();
    pos6 = atf.getTagPose(8).get().toPose2d();
        
    }
    else{
      pos1 = atf.getTagPose(18).get().toPose2d();
      pos2 = atf.getTagPose(19).get().toPose2d();
      pos3 = atf.getTagPose(20).get().toPose2d();
      pos4 = atf.getTagPose(21).get().toPose2d();
      pos5 = atf.getTagPose(22).get().toPose2d();
      pos6 = atf.getTagPose(17).get().toPose2d();
         
    }
    reefPoses.add(pos1);
    reefPoses.add(pos2);
    reefPoses.add(pos3);
    reefPoses.add(pos4);
    reefPoses.add(pos5);
    reefPoses.add(pos6);
  }

  private void SetupCoralFeedPoses(){
    
    if (isRedAlliance){
      for (int i = -3 ; i <= 3 ; i ++){
        feederStationPoses.add(atf.getTagPose(2).get().toPose2d().plus(new Transform2d(0,Units.inchesToMeters(8*i),new Rotation2d().kZero)));
        feederStationPoses.add(atf.getTagPose(1).get().toPose2d().plus(new Transform2d(0,Units.inchesToMeters(8*i),new Rotation2d().kZero)));
      }
        
    }
    else{
      for (int i = -3 ; i <= 3 ; i ++){
        feederStationPoses.add(atf.getTagPose(12).get().toPose2d().plus(new Transform2d(0,Units.inchesToMeters(8*i),new Rotation2d().kZero)));
        feederStationPoses.add(atf.getTagPose(13).get().toPose2d().plus(new Transform2d(0,Units.inchesToMeters(8*i),new Rotation2d().kZero)));
      }
    }
  }

  private void CheckZones(){

    inLeftCoralZone = WithinZone("LeftCoralStationZone",LeftCoralStationPose.toPose2d(),CurrentPose, 3, 0, 180);
    inRightCoralZone = WithinZone("RightCoralStationZone",RightCoralStationPose.toPose2d(),CurrentPose, 3, 0, 180);
    inCoralStationRedZone = WithinZone("LeftCoralStationRedZone",LeftCoralStationPose.toPose2d(),CurrentPose, 2, 0, 90) || WithinZone("RightCoralStationRedZone",RightCoralStationPose.toPose2d(),CurrentPose, 2, 0, 90);
    inReefRedZone = WithinZone("ReefRedZone_ClosestTag",ClosestReefSegment, CurrentPose, 1.25, 0, 45) || WithinZone("ReefRedZone_MainBorder",ReefPose, CurrentPose, 1.8, 0, 180) ;
    inReefYellowZone = WithinZone("ReefYelloZone",ReefPose, CurrentPose, Units.inchesToMeters(38.25)+2, 0, 180) && !inReefRedZone;
    if(inLeftCoralZone){
      inCoralStationPickupZone = WithinZone("LeftCoralStationPickupZone",ClosestPickupSlot,CurrentPose, 0.6, 180, 15);
    }
    else if(inRightCoralZone){
      inCoralStationPickupZone = WithinZone("RightCoralStationPickupZone",ClosestPickupSlot,CurrentPose, 0.6, 180, 15);
    }
    else{
      inCoralStationPickupZone = false;
    }
  }

  private void CalculateClosestReefSegment(){
    ClosestReefSegment = CurrentPose.nearest(reefPoses);
    DogLog.log("Zones/ClosestReefSegment", ClosestReefSegment);
  }

  private void CalculateClosestPickupSlot(){
    ClosestPickupSlot = CurrentPose.nearest(feederStationPoses);
    DogLog.log("Zones/ClosestPickupSlot", ClosestPickupSlot);
  }

  private Boolean WithinZone(String name, Pose2d targetPose, Pose2d currentPose, double radius, double headingOffset, double impactedHeading){

    double targetX = targetPose.getX();
    double targetY = targetPose.getY();
    double currX = currentPose.getX();
    double currY = currentPose.getY();

    double distance = Math.sqrt(Math.pow(targetY-currY,2)+Math.pow(targetX-currX,2));
    
    currentPose = currentPose.plus(new Transform2d(0,0,Rotation2d.fromDegrees(headingOffset)));
    targetPose = targetPose.plus(new Transform2d(0,0,Rotation2d.fromDegrees(180)));

    Transform2d t = new Transform2d(currentPose,targetPose);
    DogLog.log("Zones/"+name+"_currentPose", currentPose.getRotation().getDegrees());
    DogLog.log("Zones/"+name+"_targetPose", targetPose.getRotation().getDegrees());
    DogLog.log("Zones/"+name+"_t", t.getRotation().getDegrees());
    DogLog.log("Zones/"+name+"_distance", distance);
    DogLog.log("Zones/"+name+"_radius", radius);
    DogLog.log("Zones/"+name+"_impactedHeading", impactedHeading);

    // double RightCone = robotHeading + impactedHeading; // 180
    // double LeftCone = robotHeading - impactedHeading; //60
    // if (RightCone >= 180){
    //   RightCone -= 180;
    // }
    // if (LeftCone <= -180){
    //   LeftCone += 180;
    // }
    // return distance <= radius && (targetHeading <= RightCone && targetHeading >= LeftCone);
    return distance <= radius && Math.abs(t.getRotation().getDegrees()) <= impactedHeading;
  }
  private void sim()
  {
    if (Drive_SS.getCIntakeGamePiecesAmount()>0)
    {
      hasCoral = true;
      Drive_SS.stopCIntake();
    }
    if (!hasCoral)
      {
        Drive_SS.startCIntake();
      }

    // if (WithinZone(LeftCoralStationPose.toPose2d(),CurrentPose, 3, 0, 90) && !hasCoral){
      hasCoral = true;
    // }
  }
  }
    // This method will be called once per scheduler run


