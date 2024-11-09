// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */
  private final TalonFX shooterMotor;
  private double speedSetpoint;

  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.shooterMotor_ID);

    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor.setInverted(true);
  }

  public void shoot(double speed) {
    shooterMotor.setVoltage(speed);
  }

  public void stopShoot() {
    shooterMotor.setVoltage(0);
  }

  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble()*60;
  }

  public boolean ifSpeedArrive(double arriveSpeed) {
    speedSetpoint = arriveSpeed;
    if(getShooterSpeed() >= arriveSpeed) {
      return true;
    }else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/ShooterSpeed", getShooterSpeed());
    SmartDashboard.putNumber("Shooter/speedSetpoint", speedSetpoint);
    SmartDashboard.putNumber("Shooter/ShooterMoterTemp", shooterMotor.getDeviceTemp().getValueAsDouble());
  }
}
