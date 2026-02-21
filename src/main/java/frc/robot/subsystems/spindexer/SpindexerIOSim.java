package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;


public class SpindexerIOSim implements SpindexerIO{
    final TalonFX spindexerMotor = new TalonFX(0);
    final TalonFXSimState spindexerMotorSim = spindexerMotor.getSimState();
    final TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();
    final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

    private static final double kGearRatio = 10.0;
    private final DCMotorSim spindexerMotorSimModel = new DCMotorSim(
   LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
   ),
   DCMotor.getKrakenX60Foc(1)
);
    private double speed;

    public SpindexerIOSim() {
        spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
    }

    public void updateInputs(SpindexerIOInputs inputs) {;
    }

    public void run(double speed){
        spindexerMotor.getSimState();
    }

    public void startIndexing() {
       spindexerMotorSim.setRawRotorPosition(0);
       run(10);
    }

    public void stopIndexing() {
        spindexerMotorSim.setRawRotorPosition(0);
    }

}
