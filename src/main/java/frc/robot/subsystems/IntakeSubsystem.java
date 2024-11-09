// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWheel;
  private final CANSparkMax intakeArm;

  private final PIDController armPID;

  private final RelativeEncoder armEncoder;
  private final CANcoder absoluteArmEncoder;

  private ArmFeedforward armFeedforward;

  private final CANcoderConfiguration absoluteEncoderConfig;

  private double pidOutput;

  private double feedForwardOutPut;

  private double arriveAngle;

  private double outPut;

  public IntakeSubsystem() {
    intakeWheel = new TalonFX(IntakeConstants.intakeWheel_ID);
    intakeArm = new CANSparkMax(IntakeConstants.intakeArm_ID, MotorType.kBrushless);

    armPID = new PIDController(IntakeConstants.intakeArmPID_Kp, IntakeConstants.intakeArmPID_Ki, IntakeConstants.intakeArmPID_Kd);

    armEncoder = intakeArm.getEncoder();
    absoluteArmEncoder = new CANcoder(IntakeConstants.absoluteArmEncoderID);
    absoluteEncoderConfig = new CANcoderConfiguration();

    armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks1, IntakeConstants.intakeArmFeedforward_Kg1, IntakeConstants.intakeArmFeedforward_Kv1);

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.intakeCancoderOffset;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absoluteArmEncoder.getConfigurator().apply(absoluteEncoderConfig);

    arriveAngle = IntakeConstants.arriveUpAngle;


    intakeArm.restoreFactoryDefaults();

    intakeWheel.setNeutralMode(NeutralModeValue.Coast);
    intakeArm.setIdleMode(IdleMode.kBrake);

    intakeWheel.setInverted(false);
    intakeArm.setInverted(false);

    intakeArm.burnFlash();

  }

  public void noteIntake() {
    intakeWheel.setVoltage(IntakeConstants.intakewheelVoltage);
  }

  public void DownArm() {
    arriveAngle = IntakeConstants.arriveDownAngle;
  }

  public void stopIntake() {
    intakeWheel.setVoltage(0);
  }

  public void raiseArm() {
    arriveAngle = IntakeConstants.arriveUpAngle;
  }

  public void noteOut() {
    intakeWheel.setVoltage(IntakeConstants.intakeWheelEjectVoltage);
  }

  public double getAngle() {
    return absoluteArmEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getAbsolutePosition() {
    return absoluteArmEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getRelativePosition() {
    return armEncoder.getPosition();
  }

  // public void resetAbsolutedEncoder() {
  //   absoluteArmEncoder.setPosition(0);
  // }

  public double getRadians() {
    return Math.toRadians(getAngle());
  }

  public double getArmVelocity() {
    return absoluteArmEncoder.getVelocity().getValueAsDouble()*2*Math.PI;//Rpm/60*gearRatio*2PI = 角速度 radians/s
  }

  public boolean isJam() {
    return !intakeWheel.getFault_StatorCurrLimit().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getAngle() > 45) {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks6, IntakeConstants.intakeArmFeedforward_Kg6, IntakeConstants.intakeArmFeedforward_Kv6);
    }else if(32 < getAngle() && getAngle() <= 45) {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks3, IntakeConstants.intakeArmFeedforward_Kg3, IntakeConstants.intakeArmFeedforward_Kv3);
    }else if(28 < getAngle() && getAngle() <= 32) {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks1, IntakeConstants.intakeArmFeedforward_Kg1, IntakeConstants.intakeArmFeedforward_Kv1);
    }else if(0 < getAngle() && getAngle() <= 28) {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks5, IntakeConstants.intakeArmFeedforward_Kg5, IntakeConstants.intakeArmFeedforward_Kv5);
    }else if(-35 < getAngle() && getAngle() <= 0 ) {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks4, IntakeConstants.intakeArmFeedforward_Kg4, IntakeConstants.intakeArmFeedforward_Kv4);
    }else {
      armFeedforward = new ArmFeedforward(IntakeConstants.intakeArmFeedforward_Ks2, IntakeConstants.intakeArmFeedforward_Kg2, IntakeConstants.intakeArmFeedforward_Kv2);
    }
    pidOutput = armPID.calculate(getAngle(), arriveAngle);
    feedForwardOutPut = armFeedforward.calculate(getRadians(), getArmVelocity())/12;
    outPut = pidOutput + feedForwardOutPut;
    outPut = Constants.setMaxOutPut(outPut, IntakeConstants.intakeArmMaxOutPut);

    // SmartDashboard.putNumber("Intake/ArmAbsolutedEncoderPosition", getAbsolutePosition());
    // SmartDashboard.putNumber("Intake/ArmAbsoluteEncoderAngle", getAngle());
    // SmartDashboard.putNumber("Intake/ArmPidOutPut", pidOutput);
    // SmartDashboard.putNumber("Intake/ArmFeedForwardOutPut", feedForwardOutPut);
    // SmartDashboard.putNumber("Intake/ArmOutPut", outPut);
    SmartDashboard.putBoolean("Intake/ArmIsJam", isJam());
    // SmartDashboard.putNumber("Intake/ArmRelativePosition", getRelativePosition());
    SmartDashboard.putNumber("Intake/WheelMotorTemp", intakeWheel.getDeviceTemp().getValueAsDouble());

    intakeArm.set(outPut);
  }
}
