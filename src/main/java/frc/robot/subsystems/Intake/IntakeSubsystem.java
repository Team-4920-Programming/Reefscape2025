// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubSystem. */
  

     private boolean HasGP = false;
     private boolean RunIntake = false;
     private boolean IntakeSensor = false;
     private DigitalInput IntakeGPSensor; 

  public IntakeSubsystem() {
    IntakeGPSensor = new DigitalInput(0);
  }
    
  @Override
  public void periodic() {
    IntakeSensor = IntakeGPSensor.get();
    if (Robot.isReal()){
      HasGP = IntakeSensor;
    }
    else{

    }
  }
  public void IntakeOn(){
    RunIntake = true;
  }
  public void IntakeOff(){
    RunIntake = false;
  }
  public boolean getIntakeStatus(){
    return RunIntake;
  }
  public boolean getGPStatus(){
    return HasGP;
  }
  public void SetSimGP(boolean GPState)
  {
    if (Robot.isSimulation()){
      HasGP = GPState;
    }
  }
}
