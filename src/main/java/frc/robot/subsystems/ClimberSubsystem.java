// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

  /** 你的極限開關呢 
   * 還有encoder
  */
public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax leftClimberMotor;
  private final CANSparkMax rightClimberMotor;

  private final RelativeEncoder leftClimbEncoder;
  private final RelativeEncoder rightClimbEncoder;

  private final DigitalInput rightRopeFinal;
  private final DigitalInput leftRopeFinal;
  public ClimberSubsystem() {
    leftClimberMotor = new CANSparkMax(ClimberConstants.leftClimberMotor_ID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(ClimberConstants.rightClimberMotor_ID, MotorType.kBrushless);

    rightRopeFinal = new DigitalInput(ClimberConstants.leftRopeFinal_ID);
    leftRopeFinal = new DigitalInput(ClimberConstants.rightRopeFinal_ID);

    leftClimbEncoder = leftClimberMotor.getEncoder();
    rightClimbEncoder = rightClimberMotor.getEncoder();

    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    //記得到時候要測方向
    leftClimberMotor.setInverted(true);
    rightClimberMotor.setInverted(true);

    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void rightClimb(double Value) {
    if(Value > 0) {
      if(getRightPosition() < ClimberConstants.rightMaxClimbposition) {
        rightClimberMotor.setVoltage(Value*12);
      }else {
        rightClimberMotor.setVoltage(0);
      }
    }else {
      if(rightInTheEnd()) {
        resetRightPosition();
        rightClimberMotor.setVoltage(0);
      }else {
        rightClimberMotor.setVoltage(Value*12);
      }
    }
  }

  public void leftClimb(double Value) {
    if(Value > 0) {
      if(getLeftPosition() < ClimberConstants.leftmaxClimbPosition) {
        leftClimberMotor.setVoltage(Value*12);
      }else {
        leftClimberMotor.setVoltage(0);
      }
    }else {
      if(leftInTheEnd()) {
        resetLeftPosition();
        leftClimberMotor.setVoltage(0);
      }else {
        leftClimberMotor.setVoltage(Value*12);
      }
    }
  }

  public void resetRightPosition() {
    rightClimbEncoder.setPosition(0);
  }

  public void resetLeftPosition() {
    leftClimbEncoder.setPosition(0);
  }
  public double getRightPosition() {
    return rightClimbEncoder.getPosition();
  }

  public double getLeftPosition() {
    return leftClimbEncoder.getPosition();
  }
  

  public void stopClimb() {
    leftClimberMotor.setVoltage(0);
    rightClimberMotor.setVoltage(0);
  }

  public boolean rightInTheEnd() {
    return !rightRopeFinal.get();
  }

  public boolean leftInTheEnd() {
    return !leftRopeFinal.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber/leftInTheEnd?", leftInTheEnd());
    SmartDashboard.putBoolean("Climber/rightInTheEnd?", rightInTheEnd());
  }
}
