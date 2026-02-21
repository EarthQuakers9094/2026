package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class SpindexerIOSim implements SpindexerIO {
  private DCMotor spindexeDcMotor = DCMotor.getNeoVortex(1);
  private final SparkFlex spindexerMotor = new SparkFlex(0, MotorType.kBrushless);
  private final SparkFlexSim spindexerMotorSim = new SparkFlexSim(spindexerMotor, spindexeDcMotor);
  private final SparkFlexConfig spindexerMotorConfig = new SparkFlexConfig();
  private double setSpeed;

  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(1),
              Constants.SpindexerConstants.spindexerMOI,
              Constants.SpindexerConstants.spindexerGearing),
          spindexeDcMotor,
          1);

  public SpindexerIOSim() {
    spindexerMotor.configure(
        spindexerMotorConfig.apply(new ClosedLoopConfig().pid(0.1, 0, 0.1)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    flywheelSim.setInput(spindexerMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    flywheelSim.update(0.02);
    spindexerMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), 12, 0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    inputs.spindexerCurrentSpeed = setSpeed;
  }

  public void run(AngularVelocity spindexerSetSpeed) {
    spindexerMotor
        .getClosedLoopController()
        .setSetpoint(spindexerSetSpeed.in(RPM), ControlType.kVelocity);
  }
}
