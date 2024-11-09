// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
  /** Creates a new ShootSpeaker. */
  private final ShooterSubsystem m_shooterSubsystem;

  private final IndexerSubsystem m_indexerSubsystem;
  private BooleanSupplier ifFeed;
  private boolean shouldShoot;
  public ShootSpeaker(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, BooleanSupplier ifFeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_indexerSubsystem = indexerSubsystem;
    this.ifFeed = ifFeed;
    this.shouldShoot = false;

    addRequirements(m_shooterSubsystem, m_indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.shoot(ShooterConstants.shootSpeakerVoltage);

    LEDConstants.prepSPEAKER = true;
    LEDConstants.LEDFlag = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ifFeed.getAsBoolean() && shouldShoot == true){
      m_indexerSubsystem.startMotor();
    }
    if(m_shooterSubsystem.ifSpeedArrive(ShooterConstants.speedSpeaker)){
      LEDConstants.speedReadySPEAKER = true;
      LEDConstants.LEDFlag = true;
      if(ifFeed.getAsBoolean() && shouldShoot == false) {
        shouldShoot = true;
      }
    } else {
      if(shouldShoot == false) {
      m_indexerSubsystem.stopIndexer();
      }
      LEDConstants.prepSPEAKER = true;
      LEDConstants.speedReadySPEAKER = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShoot();
    m_indexerSubsystem.stopIndexer();

    if(m_indexerSubsystem.hasNote()){
      LEDConstants.hasNote = true;
    }else {
      LEDConstants.hasNote = false;
    }
    LEDConstants.speedReadySPEAKER = false;
    LEDConstants.prepSPEAKER = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}