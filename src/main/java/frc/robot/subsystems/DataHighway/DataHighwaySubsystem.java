// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.DataHighway;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.*;
public class DataHighwaySubsystem extends SubsystemBase {
  /** Creates a new DataHighwaySubsystem. */
  private int LEDPattern;
  private double timeout =0;
  private int BlinkinPWMChannnel = 0;

  enum Patterns{
    RainbowParty (1005),
    RainbowOcean (1015),
    RainbowLay (1035),
    RainbowForset(1045),
    RainbowGlitter(1055),
    ShotRed(1075),
    ShotBlue(1085),
    ShotWhite(1095),
    RainbowSinelon(1105),
    PartySinelon(1115),
    PceanSinelon(1125),
    LavaSinelon(1135),
    ForestSinelon(1145),
    RainboxBpm(1155),
    PartyBPM(1165),
    OceanBPM(1175),
    LavaBPM(1185),
    ForestBPM(1195),
    MediumFire(1205),
    LargeFire(1215),
    RainebowTwinkles(1225),
    ParyTwinkles(1235),
    OceanTwinkles(1245),
    LavaTwinkles(1255),
    ForestTwinkles(1245),
    RainbowColorWaves(1255),
    PartyColorWaves(1265),
    OceanColorWaves(1275),
    ForestColorWaves(1285),
    ResLasonScanner(1325),
    GreyLasonSanner(1335),
    RedLightChase(1345),
    BlueLightChase(1355),
    GreayLightChase(1365),
    RedHearBeat(1375),
    BlueHeartBeat(1385),
    WhiteHeartBeat(1395),
    GreyHeartBeat(1405),
    RedBreath(1415),
    BlueBreath(1425),
    GrayBreath(1435),
    RedStrobe(1445),
    BlueStrobe(1455),
    GoldStobe(1465),
    WhiteStrobe(1475),
    E2End2Black(1485),
    LarsonScanner(1495),
    LightChase(1505),
    HeartbeatSlow(1515),
    HeartbeatFast(1525),
    BreathSlow(1535),
    BreathFast(1545),
    Shot(1555),
    Stobe(1565),
    E2End2Black2 (1575),
    LarsonScanner2(1585),
    LightChase2(1595),
    HeartbeatSlow2(1615),
    HeartbeatFast2(1625),
    BreathSlow2(1635),
    BreathFast2(1645),
    Shot2(1655),
    Stobe2(1675),
    SparkleC1onC2(1685),
    SparkleC2onC1(1695),
    GradientC1C2(1705),
    BPMC1C1(1715),
    E2EC1C2(1725),
    E2EBlend(1735),
    C1C2noBlend(1745),
    C1C2Twinkles(1755),
    WavesC1C2(1765),
    SinelonC1C2(1775),
    HotPink(1785),
    DarkRed(1795),
    Red(1805),
    RedOrange(1815),
    Orange(1825),
    Gold(1835),
    Yellow(1845),
    LawnGreen(1855),
    Lime(1865),
    DarkGreen(1875),
    Green(1885),
    BlueGreen(1895),
    Aqua(1905),
    SkyBlue(1915),
    DarkBlue(1925),
    Blue(1935),
    BlueViolet(1945),
    Violet(1955),
    White(1965),
    Gray(1975),
    DarkGray(1985),
    Black(1995);
    private final int Patvalue;
    Patterns (int PatVal)
    {
      this.Patvalue = PatVal;
      
    }
  }
  private Patterns Pat;
  public boolean GPinintake = false;

  public DataHighwaySubsystem() {
  }

  @Override
  public void periodic() {
        if ((Timer.getFPGATimestamp() - timeout)> 1.0) 
      {
          Set5VStrip();
      }
      else


        if (GPinintake)
        { 
          LEDPattern = Pat.Gold.Patvalue;
        }
        else
        {
          LEDPattern = Pat.DarkRed.Patvalue;
        }
          SetStripPattern(LEDPattern);
      }

  private void SETLED(Patterns pat)
  {
    int patv = pat.Patvalue;
    DogLog.log("LEDPattern",patv);
    SetStripPattern(patv);


  }
  private void Set5VStrip()
  {
    try {
      PWMJNI.setPulseTimeMicroseconds(BlinkinPWMChannnel, 2125);
      timeout = Timer.getFPGATimestamp();
  }
     catch (Exception e) {
      // TODO: handle exception
    }
  }

    
  private void SetStripPattern(int Pattern)
  {
    try {
    PWMJNI.setPulseTimeMicroseconds(BlinkinPWMChannnel, Pattern);
  }
  catch (Exception e) {
   // TODO: handle exception
 }

  } 
    // This method will be called once per scheduler run
  }


