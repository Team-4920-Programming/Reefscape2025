// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ReefSurvey;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.RawDetection;

public class ReefSurveySubsystem extends SubsystemBase {

  public int[][] ReefScoreTracker = new int[3][12];
  private int scorethreshold = 3;
  /** Creates a new ReefSurvey. */
  public ReefSurveySubsystem() {

    

    
  }

  public void ScoreReef(int i, int j){
    ReefScoreTracker[i][j] += 1;
  }

  public boolean IsReefLocScored(int i, int j){
    return ReefScoreTracker[i][j] >= scorethreshold;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Limelight target count", LimelightHelpers.getTargetCount("limelight"));
    LimelightResults llresults;
    llresults = LimelightHelpers.getLatestResults("limelight");

    DogLog.log("Limelight target detector size", llresults.targets_Detector.length);

    for (int i = 0; i < llresults.targets_Detector.length; i++){
        LimelightTarget_Detector a = llresults.targets_Detector[i];
        DogLog.log("i = ", i);
        DogLog.log(a.className +" " + i + " classid", a.classID);
        DogLog.log(a.className +" " + i + " tx", a.tx);
        DogLog.log(a.className +" " + i + " ty", a.ty);
        DogLog.log(a.className +" " + i + " ta", a.ta);
        DogLog.log(a.className +" " + i + " confidence", a.confidence);
    }
  }

}
