// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNote_LimeLight extends Command {
  /** Creates a new TrackNote_LimeLight. */
  private final SwerveSubsystem m_swerveSubsystem;
  private final LimeLightSubsystem m_limLightSubsystem;


  private double pidOutPut;
  public TrackNote_LimeLight(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limLightSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_limLightSubsystem = limLightSubsystem;

    addRequirements(m_swerveSubsystem, m_limLightSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidOutPut = 0;

    LEDConstants.trackingNote = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidOutPut = m_limLightSubsystem.getPidOutPut();
    SmartDashboard.putNumber("NoteDetectionPidOutPut", pidOutPut);
    if(m_limLightSubsystem.hasNote()){
      LEDConstants.hasNoteInSight = true;
      LEDConstants.LEDFlag = true;
      if(Math.abs(m_limLightSubsystem.getPidPositionError())>5){
        m_swerveSubsystem.drive(0, 0, pidOutPut, false);
      }else{
        m_swerveSubsystem.drive(0.4, 0, 0, false);
      }
  }else {
    LEDConstants.hasNoteInSight = false;
    LEDConstants.LEDFlag = true;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.trackingNote = false;
    LEDConstants.hasNoteInSight = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LEDConstants.hasNote;
  }
}
