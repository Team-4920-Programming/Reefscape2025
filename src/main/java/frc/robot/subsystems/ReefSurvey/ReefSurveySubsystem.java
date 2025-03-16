// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ReefSurvey;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.RawDetection;

public class ReefSurveySubsystem extends SubsystemBase {

  public int[][] ReefScoreTracker = new int[4][12];
  private int scorethreshold = 1;
  public int DH_In_ReefSegment = 0;
  public int DH_In_ScoreSelection = 0;
  /** Creates a new ReefSurvey. */
  public ReefSurveySubsystem() {

    

    
  }

  public void ScoreReef(int l, int b){
    System.out.println ( "ReefScore" + ReefScoreTracker[l][b] + " " + l + " " +b);
    ReefScoreTracker[l][b] += 1;
  }

  public boolean IsReefLocScored(int i, int j){
    //System.out.println ( "ReefScore" + ReefScoreTracker[i][j] + " " + i + " " +j);
    return ReefScoreTracker[i][j] >= scorethreshold;
  }
  public char getReefDataScoredChar(int i, int j)
  {
    if (IsReefLocScored(i, j))
      return '1';
    else
      return '0';
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
    String reefdata = "000000000000000000000000000000000000"; //36 positions
        //Eric uses segment 1 to 6, and postion 1 to 12 in that segement (L2 Left to L4 Right).
        
        char[] ReefChar = reefdata.toCharArray();
        ReefChar[0] = getReefDataScoredChar(0,0); //seg 1 level 2L
        ReefChar[1] = getReefDataScoredChar(0,1); //seg 1 Level 2R
        ReefChar[2] = getReefDataScoredChar(1, 0); // set 1 Level 3L
        ReefChar[3] = getReefDataScoredChar(1, 1); //Seg 1 Level 3R
        ReefChar[4]= getReefDataScoredChar(2, 0); // set 1 Level 4L
        ReefChar[5]= getReefDataScoredChar(2, 1); // set 1 Level 4R

        ReefChar[6] = getReefDataScoredChar(0,2); //seg 2 level 2L
        ReefChar[7] = getReefDataScoredChar(0,3); //seg 2 Level 2R
        ReefChar[8] = getReefDataScoredChar(1, 2); // set 2 Level 3L
        ReefChar[9] = getReefDataScoredChar(1, 3); //Seg 2 Level 3R
        ReefChar[10]= getReefDataScoredChar(2, 2); // set 2 Level 4L
        ReefChar[11]= getReefDataScoredChar(2, 3); // set 2 Level 4R

        ReefChar[12] = getReefDataScoredChar(0,4); //seg 3 level 2L
        ReefChar[13] = getReefDataScoredChar(0,5); //seg 3 Level 2R
        ReefChar[14] = getReefDataScoredChar(1, 4); // set 3 Level 3L
        ReefChar[15] = getReefDataScoredChar(1, 5); //Seg 3 Level 3R
        ReefChar[16]= getReefDataScoredChar(2, 4); // set 3 Level 4L
        ReefChar[17]= getReefDataScoredChar(2, 5); // set 3 Level 4R

        ReefChar[18] = getReefDataScoredChar(0,6); //seg 4 level 2L
        ReefChar[19] = getReefDataScoredChar(0,7); //seg 4 Level 2R
        ReefChar[20] = getReefDataScoredChar(1, 6); // set 4 Level 3L
        ReefChar[21] = getReefDataScoredChar(1, 7); //Seg 4 Level 3R
        ReefChar[22]= getReefDataScoredChar(2, 6); // set 4 Level 4L
        ReefChar[23]= getReefDataScoredChar(2, 7); // set 4 Level 4R

        ReefChar[24] = getReefDataScoredChar(0,8); //seg 5 level 2L
        ReefChar[25] = getReefDataScoredChar(0,9); //seg 5 Level 2R
        ReefChar[26] = getReefDataScoredChar(1, 8); // set 5 Level 3L
        ReefChar[27] = getReefDataScoredChar(1, 9); //Seg 5 Level 3R
        ReefChar[28]= getReefDataScoredChar(2, 8); // set 5 Level 4L
        ReefChar[29]= getReefDataScoredChar(2, 9); // set 5 Level 4R
   
        ReefChar[30] = getReefDataScoredChar(0,10); //seg 6 level 2L
        ReefChar[31] = getReefDataScoredChar(0,11); //seg 6 Level 2R
        ReefChar[32] = getReefDataScoredChar(1, 10); // set 6 Level 3L
        ReefChar[33] = getReefDataScoredChar(1, 11); //Seg 6 Level 3R
        ReefChar[34]= getReefDataScoredChar(2, 10); // set 6 Level 4L
        ReefChar[35]= getReefDataScoredChar(2,11); // set 6 Level 4R

        String reefDataString = String.valueOf(ReefChar);
        SmartDashboard.putString("ReefString", reefDataString);
        DogLog.log("ReefString", reefDataString); //publish so Driverstation can read
       
  }

}
