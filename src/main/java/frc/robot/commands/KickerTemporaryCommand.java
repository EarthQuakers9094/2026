// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerSubsystem;

public class KickerTemporaryCommand extends Command {
  private final KickerSubsystem subsystem;

  public KickerTemporaryCommand(KickerSubsystem subsystem) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.startKicker();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopKicker();
  }

  private boolean shouldEnd() {
    /*double velocity = subsystem.getRPM();
    double setpoint = subsystem.getRPMSetpoint();
    double five_percent_setpoint = setpoint / 20d;
    if (velocity >= (setpoint - five_percent_setpoint)
        && velocity <= (setpoint + five_percent_setpoint)) {
      return true;
    }*/
    return false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd();
  }
}
