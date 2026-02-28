package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class KickerIOReal implements KickerIO {
  private final TalonFX motor;
  private AngularVelocity velocitySetpoint = RPM.of(0.0d);

  public KickerIOReal() {
    motor = new TalonFX(Constants.KickerConstants.motorId, Constants.shooterCANBus);
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.KickerConstants.kP)
                .withKI(Constants.KickerConstants.kI)
                .withKD(Constants.KickerConstants.kD)
                .withKV(Constants.KickerConstants.kV));
  }

  public void startKicker() {
    motor.setControl(new VelocityVoltage(Constants.KickerConstants.velocitySetpoint));
    velocitySetpoint = Constants.KickerConstants.velocitySetpoint;
  }

  public void stopKicker() {
    motor.setControl(new VelocityVoltage(0));
    velocitySetpoint = RPM.of(0.0d);
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.angularVelocity = RotationsPerSecond.of(motor.getVelocity().getValueAsDouble());
    inputs.angularVelocitySetpoint = velocitySetpoint;
  }
}

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
