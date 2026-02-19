package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class KickerIOSim implements KickerIO {
  private final SparkFlexSim motorSIM;
  private final SparkFlex motor;
  private final DCMotor dcVortex;
  private final FlywheelSim flywheelSIM;

  public KickerIOSim() {
    dcVortex = DCMotor.getNeoVortex(1);

    flywheelSIM =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                dcVortex,
                Constants.KickerConstants.simFlywheelMOI,
                Constants.KickerConstants.simFlywheelGearing),
            dcVortex);

    motor = new SparkFlex(Constants.KickerConstants.motorId, MotorType.kBrushless);
    motor.configure(
        new SparkFlexConfig()
            .inverted(false)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(Constants.KickerConstants.encoder_conversion_factor))
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
    motorSIM = new SparkFlexSim(motor, dcVortex);
  }

  public void startKicker() {
    motor
        .getClosedLoopController()
        .setSetpoint(Constants.KickerConstants.activeVelocity, ControlType.kVelocity);
  }

  public void stopKicker() {
    motor.getClosedLoopController().setSetpoint(0.0d, ControlType.kVelocity);
  }

  public double getVelocityMeters() {
    return motor.getEncoder().getVelocity();
  }

  public double getVelocitySetpointMeters() {
    return motor.getClosedLoopController().getSetpoint();
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.current_velocity_in_meters = getVelocityMeters();
    inputs.velocity_sepoint_in_meters = getVelocityMeters();
  }
}
