package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.security.AllPermission;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDEvent {
    StartedAuto,
    StartedShooting,
    HubActive,
    HubInactive,
    ClapBoard
  }

  private static Queue<LEDEvent> events = new LinkedList<>();

  @AutoLogOutput
  private Optional<LEDEvent> maybeCurrentEvent = Optional.empty();
  private double remainingEventTime = 0.0;

  private final CANdle candle;

  private boolean firstIteration = true;
  private double lastTime = Timer.getFPGATimestamp();

  private static final int length = 150;

  public LEDSubsystem() {
    this.candle = new CANdle(3);

    // Logger.recordOutput("ClapBoard", 1.0);
    // candle.setControl(new SolidColor(0, 150).withColor(RGBWColor.fromHSV(null, lastTime, remainingEventTime)));

    firstIteration = true;
  }

  // @Override
  // public void initialize() {}

  @Override
  public void periodic() {
    if (firstIteration) {
      Logger.recordOutput("ClapBoard", 0.0);
      returnToDefault();
      firstIteration = false;
    }

    double newTime = Timer.getFPGATimestamp();
    double dt = newTime - lastTime;
    lastTime = newTime;

    remainingEventTime -= dt;

    if (remainingEventTime <= 0) {
      if (events.isEmpty()) {
        returnToDefault();
      } else {
        transitionToEvent(events.remove());
      }
    }
  }

  public static void sendEvent(LEDEvent event) {
    events.add(event);
  }
  

  private void returnToDefault() {
        Logger.recordOutput("ClapBoard", 0.0);

    candle.setControl(new LarsonAnimation(0, length).withColor(getAllianceColor(getAlliance())));
    this.maybeCurrentEvent = Optional.empty();
  }

  private void transitionToEvent(LEDEvent event) {
    remainingEventTime = 1.5;
    this.maybeCurrentEvent = Optional.of(event);

    switch (event) {
      case ClapBoard:
        Logger.recordOutput("ClapBoard", 1.0);
        candle.setControl(new SolidColor(0, length).withColor(new RGBWColor(209, 71, 191)));
        break;
      case HubActive:
        candle.setControl(new StrobeAnimation(0, length).withColor(getAllianceColor(getAlliance())));
        break;
      case HubInactive:
        candle.setControl(new StrobeAnimation(0, length).withColor(getAllianceColor(getOtherAlliance())));
        break;
      case StartedAuto:
        candle.setControl(new StrobeAnimation(0, length).withColor(new RGBWColor(34, 156, 63)));
        break;
      case StartedShooting:
        candle.setControl(new StrobeAnimation(0, length).withColor(new RGBWColor(227, 227, 61)));
        break;
      default:
        break;
      
    }
  }

  private static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private static Alliance getOtherAlliance() {
    switch (getAlliance()) {
      case Blue:
        return Alliance.Red;
      case Red:
        return Alliance.Blue;
      default:
        return Alliance.Blue;
      
    }
  }

  private static RGBWColor getAllianceColor(Alliance alliance) {
    switch (alliance) {
      case Blue:
        return new RGBWColor(0, 0, 255);
      case Red:
        return new RGBWColor(255, 0, 0);
      default:
        return new RGBWColor(255, 0, 255);
      
    }
  }

}
