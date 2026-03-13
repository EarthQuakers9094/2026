package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDEvent {}

  public static Queue<LEDEvent> events = new LinkedList<>();

  private final CANdle candle;

  public LEDSubsystem() {
    this.candle = new CANdle(3);

    Logger.recordOutput("ClapBoard", 1.0);
    candle.setControl(new SolidColor(0, 150));
  }

  // @Override
  // public void initialize() {}

  @Override
  public void periodic() {
    Logger.recordOutput("ClapBoard", 0.0);
    candle.setControl(new SolidColor(0, 150));
  }
}