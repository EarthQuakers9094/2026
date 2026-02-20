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
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
                    // TODO make a conversion factor, or ask a cadder, that works consistently with
                    // the internal RPM angular velocity.
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
        .setSetpoint(Constants.KickerConstants.activeRPM, ControlType.kVelocity);
  }

  public void stopKicker() {
    motor.getClosedLoopController().setSetpoint(0.0d, ControlType.kVelocity);
  }

  public double getRPM() {
    return flywheelSIM.getAngularVelocityRPM();
  }

  public double getRPMSetpoint() {
    return motor.getClosedLoopController().getSetpoint();
  }

  public void updateInputs(KickerIOInputs inputs) {
    // TODO, ask CHARLIE about Battery SIM as he has it set and I dont know if i should change it
    /*RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSIM.getCurrentDrawAmps())
    );*/

    double motorVoltage = motorSIM.getAppliedOutput() * RoboRioSim.getVInVoltage();

    flywheelSIM.setInputVoltage(motorVoltage);
    flywheelSIM.update(0.020);

    double rpm = getRPM();

    motorSIM.iterate(rpm, RoboRioSim.getVInVoltage(), 0.020);

    inputs.rpm_present = getRPM();
    inputs.rpm_setpoint = getRPMSetpoint();
  }
}
