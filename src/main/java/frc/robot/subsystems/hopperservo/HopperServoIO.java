package frc.robot.subsystems.hopperservo;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HopperServoIO {
  @AutoLog
  public static class HopperServoIOInputs {
    public Angle servoAngle = Degrees.of(0);
    public Angle servoAngleSetpoint = Degrees.of(0);
  }

  public default void updateInputs(HopperServoIOInputs inputs) {}

  public default double getPWMPos() {
    return 0.0;
  }

  public default void setSetpointPWM(double PWM) {}

  public default void setSetpoint(Angle setpoint) {}
}
