// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class VerticalMovement extends Command {
  /** Creates a new climbUp. */
  private final ClimberSubsystem m_climberSubsystem;
  private DoubleSupplier leftClimbSpeedFunc;
  private DoubleSupplier rightClimbSpeedFunc;

  private double leftClimbSpeed;
  private double rightClimbSpeed;

  private BooleanSupplier climberInsurance;
  public VerticalMovement(ClimberSubsystem climberSubsystem, DoubleSupplier leftClimbSpeedFunc, DoubleSupplier rightClimbSpeedFunc, BooleanSupplier climberInsurance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climberSubsystem = climberSubsystem; 
    this.leftClimbSpeedFunc = leftClimbSpeedFunc;
    this.rightClimbSpeedFunc = rightClimbSpeedFunc;
    this.climberInsurance = climberInsurance;

    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightClimbSpeed = -rightClimbSpeedFunc.getAsDouble();
    leftClimbSpeed =-leftClimbSpeedFunc.getAsDouble();

    if(climberInsurance.getAsBoolean()){
      m_climberSubsystem.leftClimb(leftClimbSpeed);
      m_climberSubsystem.rightClimb(rightClimbSpeed);
    }else {
      m_climberSubsystem.leftClimb(0);
      m_climberSubsystem.rightClimb(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
