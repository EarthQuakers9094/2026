package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class KickerIOReal implements KickerIO {
  private final TalonFX motor;

  public KickerIOReal() {
    motor = new TalonFX(Constants.KickerConstants.motorId, "Shooter");
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.KickerConstants.kP)
                .withKI(Constants.KickerConstants.kI)
                .withKD(Constants.KickerConstants.kD)
                .withKV(Constants.KickerConstants.kV));
    // motor = new SparkFlex(Constants.KickerConstants.motorId, MotorType.kBrushless);
    // motor.configure(
    //     new SparkFlexConfig()
    //         .inverted(false)
    //         .apply(
    //             new EncoderConfig()
    //                 .velocityConversionFactor(Constants.KickerConstants.encoderConversionFactor))
    //         .apply(
    //             new ClosedLoopConfig()
    //                 .pid(
    //                     Constants.KickerConstants.kP,
    //                     Constants.KickerConstants.kI,
    //                     Constants.KickerConstants.kD)
    //                 .apply(
    //                     new FeedForwardConfig()
    //                         .kS(Constants.KickerConstants.kS)
    //                         .kV(Constants.KickerConstants.kV))),
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);
  }

  public void startKicker() {
    motor.setControl(new VelocityVoltage(Constants.KickerConstants.velocitySetpoint));
    Logger.recordOutput(
        "Kicker/SetpointRadPerSec",
        Constants.KickerConstants.velocitySetpoint.in(RadiansPerSecond));
    // motor
    //     .getClosedLoopController()
    //     .setSetpoint(Constants.KickerConstants.velocitySetpoint.in(RPM), ControlType.kVelocity);
  }

  public void stopKicker() {
    motor.setControl(new VelocityVoltage(0));
    Logger.recordOutput("Kicker/SetpointRadPerSec", 0);
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.angularVelocity = RotationsPerSecond.of(motor.getVelocity().getValueAsDouble());

    // inputs.angularVelocitySetpoint = RPM.of(Constants.KickerConstants.velocitySetpoint);
  }
}
