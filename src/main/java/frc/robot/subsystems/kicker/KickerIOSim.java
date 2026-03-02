package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class KickerIOSim implements KickerIO {
  private final TalonFXSimState motorSIM;
  private final TalonFX motor;
  private final DCMotor dcMotor;
  private final FlywheelSim flywheelSIM;
  private AngularVelocity velocitySetpoint = RPM.of(0.0d);

  public KickerIOSim() {
    dcMotor = DCMotor.getKrakenX60(1);

    flywheelSIM =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                dcMotor,
                Constants.KickerConstants.simFlywheelMOI,
                Constants.KickerConstants.flywheelGearing),
            dcMotor);

    motor = new TalonFX(Constants.KickerConstants.motorId, Constants.shooterCANBus);
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.KickerConstants.kP)
                .withKI(Constants.KickerConstants.kI)
                .withKD(Constants.KickerConstants.kD)
                .withKS(Constants.KickerConstants.kS)
                .withKV(Constants.KickerConstants.kVSim));

    motorSIM = new TalonFXSimState(motor);
  }

  public void startKicker() {
    motor.setControl(new VelocityVoltage(Constants.KickerConstants.simSetpoint));
    velocitySetpoint = Constants.KickerConstants.simSetpoint;
  }

  public void stopKicker() {
    motor.setControl(new VelocityVoltage(RPM.of(0.0d)));
    velocitySetpoint = RPM.of(0.0d);
  }

  public void updateInputs(KickerIOInputs inputs) {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSIM.getCurrentDrawAmps()));

    double motorVoltage = motorSIM.getMotorVoltage();

    flywheelSIM.setInputVoltage(motorVoltage);
    flywheelSIM.update(0.020);

    AngularVelocity rpm = flywheelSIM.getAngularVelocity();

    Logger.recordOutput("testing", flywheelSIM.getAngularVelocityRPM());

    motorSIM.setRotorVelocity(rpm);
    Logger.recordOutput("testing current", motor.getSupplyCurrent().getValueAsDouble());

    inputs.angularVelocity = rpm;
    inputs.angularVelocitySetpoint = velocitySetpoint;
  }
}
