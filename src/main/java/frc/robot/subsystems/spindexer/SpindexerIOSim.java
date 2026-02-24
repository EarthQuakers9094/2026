package frc.robot.subsystems.spindexer;

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

public class SpindexerIOSim implements SpindexerIO {
  private DCMotor dcMotor = DCMotor.getNeoVortex(1);
  private final SparkFlex spindexerMotor =
      new SparkFlex(Constants.SpindexerConstants.spindexerMotorId, MotorType.kBrushless);
  private final SparkFlexSim spindexerMotorSim = new SparkFlexSim(spindexerMotor, dcMotor);
  private final SparkFlexConfig spindexerMotorConfig = new SparkFlexConfig();

  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              dcMotor,
              Constants.SpindexerConstants.spindexerMOI,
              Constants.SpindexerConstants.spindexerGearing),
          dcMotor);

  public SpindexerIOSim() {
    spindexerMotor.configure(
        spindexerMotorConfig
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.SpindexerConstants.kP,
                        Constants.SpindexerConstants.kI,
                        Constants.SpindexerConstants.kD)
                    .apply(new FeedForwardConfig().kV(Constants.SpindexerConstants.kV)))
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    flywheelSim.setInput(spindexerMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    flywheelSim.update(0.02);
    spindexerMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), 12, 0.02);
    inputs.spindexerCurrentSpeed = flywheelSim.getAngularVelocity();
    inputs.spinexerVelocitySetpoint =
        RPM.of(spindexerMotor.getClosedLoopController().getSetpoint());
  }

  public void run(AngularVelocity spindexerSetSpeed) {
    spindexerMotor
        .getClosedLoopController()
        .setSetpoint(spindexerSetSpeed.in(RPM), ControlType.kVelocity);
  }
}
