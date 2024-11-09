// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LEDConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new indexerSubsystem. */
  private boolean startTime;
  private final Timer timer;
  private double nowTime;
  private final TalonFX indexerMotor;

  private final DigitalInput irSensor;
  public IndexerSubsystem() {
    timer = new Timer();
    startTime = true;

    indexerMotor = new TalonFX(IndexerConstants.indexerMotor_ID);

    irSensor = new DigitalInput(IndexerConstants.irSensor_ID);

    indexerMotor.setInverted(true);
    indexerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void startMotor() {
    indexerMotor.setVoltage(IndexerConstants.indexerVoltage);
  }

  public void outNote() {
    indexerMotor.setVoltage(-IndexerConstants.indexerVoltage);
  }
  
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }

  public boolean getBottomLimitSwitch(){
    return !irSensor.get();
  }

  // public boolean hasNote(){
  //   nowTime = timer.get();
  //   if(getBottomLimitSwitch() && startTime == true){
  //     timer.reset();
  //     timer.start();
  //     startTime = false;
  //   }
  //   if(getBottomLimitSwitch() && nowTime >= 0.05){
  //     LEDConstants.hasNote = true;
  //     timer.stop();
  //     return true;
  //   }else{
  //     startTime = true;
  //     nowTime = 0;
  //     return false;
  //   }
  // }

  //如果上面那個可以用就把這個刪掉
  public boolean hasNote(){
    if(irSensor.get() && startTime == true){
      timer.reset();
      timer.start();
      startTime = false;
    }
    nowTime = timer.get();
    if(!irSensor.get() && nowTime >= 0){
      LEDConstants.hasNote = true;
      timer.stop();
      return true;
    }else{
      startTime = true;
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getBottomLimitSwitch();
    hasNote();
    SmartDashboard.putBoolean("Indexer/hasNote", hasNote());
  }
}
