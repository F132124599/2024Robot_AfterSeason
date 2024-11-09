// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends Command {
  /** Creates a new StopIntake. */
  private final IntakeSubsystem m_IntakeSubsystem;
  public StopIntake(IntakeSubsystem intakeSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = intakeSubsytem;

    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.raiseArm();
    m_IntakeSubsystem.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
