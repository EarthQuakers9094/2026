package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.Queue;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDEvent {}

  public static Queue<LEDEvent> events = new LinkedList<>();

  public LEDSubsystem() {}
}
