// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPBarSubsystem;

public class AMPBar extends Command {
  /** Creates a new AMPBarOut. */

  private final AMPBarSubsystem m_AMPBarSubsystem;

  public AMPBar(AMPBarSubsystem AMPBarSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_AMPBarSubsystem = AMPBarSubsystem;

    addRequirements(m_AMPBarSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AMPBarSubsystem.setOutAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AMPBarSubsystem.setBackAngle();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
