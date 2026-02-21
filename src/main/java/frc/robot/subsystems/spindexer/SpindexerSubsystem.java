package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerIO spindexerIO;
  private final SpindexerIOInputsAutoLogged autoLogged = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerIO spindexerIO) {
    this.spindexerIO = spindexerIO;
  }

  public void run(AngularVelocity spindexerSpeed) {
    spindexerIO.run(spindexerSpeed);
  }

  public void stop() {
    spindexerIO.run(RPM.of(0));
  }
}
