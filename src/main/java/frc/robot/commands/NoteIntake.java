// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteIntake extends Command {
  /** Creates a new NoteIntake. */
  private final IntakeSubsystem m_intakeSubsystem;

  private final IndexerSubsystem m_indexerSubsystem;

  public NoteIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_indexerSubsystem = indexerSubsystem;

    addRequirements(m_intakeSubsystem, m_indexerSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerSubsystem.startMotor();
    m_intakeSubsystem.DownArm();
    m_intakeSubsystem.noteIntake();

    LEDConstants.intaking = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_indexerSubsystem.stopIndexer();
    m_intakeSubsystem.raiseArm();

    if(m_indexerSubsystem.hasNote()){
      LEDConstants.hasNote = true;
    }else{
      LEDConstants.hasNote = false;
    }
    LEDConstants.intaking = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.hasNote();
  }
}
