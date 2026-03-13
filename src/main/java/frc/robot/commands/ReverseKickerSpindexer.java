// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseKickerSpindexer extends Command {
  private final KickerSubsystem kicker;
  private final SpindexerSubsystem spindexer;

  public ReverseKickerSpindexer(KickerSubsystem kicker, SpindexerSubsystem spindexer) {
    this.kicker = kicker;
    this.spindexer = spindexer;

    addRequirements(kicker, spindexer);
  }

  @Override
  public void initialize() {
    kicker.reverseKicker();
    spindexer.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopKicker();
    spindexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
