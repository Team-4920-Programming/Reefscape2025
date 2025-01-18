// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubSystem. */
  

     private boolean HasAlgaeGP = false;
     private boolean RunIntake = false;
     private boolean IntakeSensor = false;
     private DigitalInput IntakeAlgaeSensor; 

  public AlgaeIntakeSubsystem() {
    IntakeAlgaeSensor = new DigitalInput(0);
  }
    
  @Override
  public void periodic() {
    IntakeSensor = IntakeAlgaeSensor.get();
    if (Robot.isReal()){
      HasAlgaeGP = IntakeSensor;
    }
    else{

    }
  }
  public void AlgaeIntakeOn(){
    RunIntake = true;
  }
  public void AlgaeIntakeOff(){
    RunIntake = false;
  }
  public boolean getAlgaeIntakeStatus(){
    return RunIntake;
  }
  public boolean getAlgaeGPStatus(){
    return HasAlgaeGP;
  }
  public void SetSimAlgaeGP(boolean GPState)
  {
    if (Robot.isSimulation()){
    }
  }
}
