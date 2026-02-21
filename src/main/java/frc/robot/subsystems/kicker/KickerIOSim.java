package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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
                Constants.KickerConstants.flywheelGearing),
            dcVortex);

    motor = new SparkFlex(Constants.KickerConstants.motorId, MotorType.kBrushless);
    motor.configure(
        new SparkFlexConfig()
            .inverted(false)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(Constants.KickerConstants.encoderConversionFactorRPM))
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
        .setSetpoint(Constants.KickerConstants.velocitySetpoint.in(RPM), ControlType.kVelocity);
  }

  public void stopKicker() {
    motor.getClosedLoopController().setSetpoint(0.0d, ControlType.kVelocity);
  }

  public void updateInputs(KickerIOInputs inputs) {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSIM.getCurrentDrawAmps()));

    double motorVoltage = motorSIM.getAppliedOutput() * RoboRioSim.getVInVoltage();

    flywheelSIM.setInputVoltage(motorVoltage);
    flywheelSIM.update(0.020);

    AngularVelocity rpm = flywheelSIM.getAngularVelocity();

    motorSIM.iterate(rpm.in(RPM), RoboRioSim.getVInVoltage(), 0.020);

    inputs.angularVelocityCurrent = rpm;
    inputs.angularVelocitySetpoint = RPM.of(motor.getClosedLoopController().getSetpoint());
  }
}
