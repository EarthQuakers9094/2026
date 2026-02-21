package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class KickerIOReal implements KickerIO {
  private final SparkFlex motor;

  public KickerIOReal() {
    motor = new SparkFlex(Constants.KickerConstants.motorId, MotorType.kBrushless);
    motor.configure(
        new SparkFlexConfig()
            .inverted(false)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(Constants.KickerConstants.encoderConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.KickerConstants.kP,
                        Constants.KickerConstants.kI,
                        Constants.KickerConstants.kD)
                    .apply(
                        new FeedForwardConfig()
                            .kS(Constants.KickerConstants.kS)
                            .kV(Constants.KickerConstants.kV))),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void startKicker() {
    motor
        .getClosedLoopController()
        .setSetpoint(Constants.KickerConstants.velocitySetpoint.in(RPM), ControlType.kVelocity);
  }

  public void stopKicker() {
    motor.getClosedLoopController().setSetpoint(0.0d, ControlType.kVelocity);
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.angularVelocityCurrent = RPM.of(motor.getEncoder().getVelocity());
    inputs.angularVelocitySetpoint = RPM.of(motor.getClosedLoopController().getSetpoint());
  }
}
