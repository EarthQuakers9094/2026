package frc.robot.subsystems.hopperservo;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class HopperServoIOReal implements HopperServoIO {
  private final Servo servo;
  private Angle setpoint = Degrees.of(0);

  public HopperServoIOReal() {
    servo = new Servo(Constants.ServoConstants.PWMPort);
  }

  public void setSetpoint(Angle setpoint) {
    this.setpoint = setpoint;
    double pwmPosition =
        (setpoint.in(Degrees) / Constants.ServoConstants.servoMaxAngle.in(Degrees));
    if (pwmPosition > 1.0) {
      pwmPosition = 1.0;
    } else if (pwmPosition < 0.0) {
      pwmPosition = 0.0;
    }
    servo.setPosition(pwmPosition);
  }

  public void updateInputs(HopperServoIOInputs inputs) {
    inputs.servoAngle =
        Degrees.of((servo.get() * Constants.ServoConstants.servoMaxAngle.in(Degrees)));
    inputs.servoAngleSetpoint = setpoint;
  }
}
