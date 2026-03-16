package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDEvent;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class GameState {
  public enum Phase {
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    Auto,
    WaitingForFMS,
    Endgame,
    NotInGame,
  }

  private static GameState instance;

  public static GameState getInstance() {
    if (instance == null) {
      instance = new GameState();
    }
    return instance;
  }

  @AutoLogOutput private Phase currentPhase = Phase.NotInGame;
  private Optional<Boolean> isFirstActiveAlliance = Optional.empty();
  private Optional<Boolean> isHubActive = Optional.empty();

  private double remainingPhaseTime = 0;
  private double lastTime = Timer.getFPGATimestamp();

  public GameState() {
    if (DriverStation.getMatchType() != MatchType.None) {
      transitionToPhase(Phase.Auto);
      lastTime = Timer.getFPGATimestamp();
    }
  }

  public void update() {

    double newTime = Timer.getFPGATimestamp();
    double dt = newTime - lastTime;
    lastTime = newTime;

    if (DriverStation.isEnabled() || currentPhase == Phase.WaitingForFMS) {
      remainingPhaseTime -= dt;
    }

    if (currentPhase == Phase.WaitingForFMS) {
      String gameData = DriverStation.getGameSpecificMessage();

      if (!gameData.isEmpty()) {
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        switch (gameData.charAt(0)) {
          case 'B':
            isFirstActiveAlliance = Optional.of(!isBlueAlliance); // Blue alliance
            // won auto
            transitionToPhase(Phase.Transition);
            break;
          case 'R':
            isFirstActiveAlliance = Optional.of(isBlueAlliance); // Red alliance won
            // auto
            transitionToPhase(Phase.Transition);
            break;
          default:
            if (remainingPhaseTime <= 0.0) {
              DriverStation.reportError("No FMS Data", false);
              LEDSubsystem.sendEvent(LEDEvent.NoFMSData);
            }
            break;
        }
      }
      return;
    }

    if (remainingPhaseTime <= 0) {
      Phase nextPhase = null;
      switch (currentPhase) {
        case Transition:
          nextPhase = Phase.Shift1;
          break;
        case Shift1:
          nextPhase = Phase.Shift2;
          break;
        case Shift2:
          nextPhase = Phase.Shift3;
          break;
        case Shift3:
          nextPhase = Phase.Shift4;
          break;
        case Shift4:
          nextPhase = Phase.Endgame;
          break;
        case Auto:
          nextPhase = Phase.WaitingForFMS;
          break;
        case WaitingForFMS:
          nextPhase = Phase.Shift1;
          break;
        case Endgame:
          nextPhase = Phase.NotInGame;
          break;
        case NotInGame:
          return;
      }
      transitionToPhase(nextPhase);
    }
  }

  private void transitionToPhase(Phase nextPhase) {
    double phaseDuration = 0;
    switch (nextPhase) {
      case Transition:
        phaseDuration = 10;
        isHubActive = Optional.of(true);
        break;

      case Shift2:
      case Shift4:
        isHubActive = isFirstActiveAlliance.map((it) -> !it); // You will be active in shift 2
        // or 4
        // if you weren't the first active
        // alliance.
        if (isHubActive.isPresent() && isHubActive.get()) {
          LEDSubsystem.sendEvent(LEDEvent.HubActive);
        }
        phaseDuration = 25;
        break;
      case Shift1:
      case Shift3:
        isHubActive = isFirstActiveAlliance; // You will be active in shift 1 or 3
        // if you are the first active
        // alliance.
        if (isHubActive.isPresent() && isHubActive.get()) {
          LEDSubsystem.sendEvent(LEDEvent.HubActive);
        }
        phaseDuration = 25;
        break;
      case Auto:
        isHubActive = Optional.of(true);
        phaseDuration = 20;
        break;
      case WaitingForFMS:
        isHubActive = Optional.of(false);
        phaseDuration = 3.1; // Give FMS a couple extra iterations to actual give us data.
        break;
      case Endgame:
        isHubActive = Optional.of(true);
        phaseDuration = 30;
        break;
      case NotInGame:
        isHubActive = Optional.of(true);
        return;
    }
    remainingPhaseTime = phaseDuration;
    currentPhase = nextPhase;
  }

  public Phase getPhase() {
    return this.currentPhase;
  }

  @AutoLogOutput
  public boolean shouldShoot(double TOF) {
    if (!isHubActive.isPresent()) {
      return false;
    }

    boolean hubActive = isHubActive.get();
    double arrivalTime = remainingPhaseTime - TOF;
    if (!hubActive) {
      return arrivalTime <= 1.0; // ~1-2 second is 6328's measured fuel processing time
    }
    return true;
  }
}
