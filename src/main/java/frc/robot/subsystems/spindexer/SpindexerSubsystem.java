package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerIO spindexerIO;
  private final SpindexerIOInputsAutoLogged autoLogged = new SpindexerIOInputsAutoLogged;

  public SpindexerSubsystem(SpindexerIO spindexerIO) {
    this.spindexerIO = spindexerIO;
  }

  public void setVelocitySetpoint(double spindexerSpeed) {
    spindexerIO.setVelocitySetpoint(spindexerSpeed);
  }
}
