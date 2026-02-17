package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class IntakeIOSim implements IntakeIO {

  DCMotor spinGearbox = DCMotor.getNeoVortex(1);
  DCMotor pivotGearbox = DCMotor.getNeoVortex(1);


  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.intakePivotMotor);
  private final TalonFX spinMotor = new TalonFX(IntakeConstants.intakeSpinMotor);

  private final TalonFXSimState pivotMotorSim;
  private final TalonFXSimState intakeMotorSim;



  private SingleJointedArmSim armSim =
    new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(
      pivotGearbox,
      Constants.IntakeConstants.intakeMOI,
      Constants.IntakeConstants.intakeGearing), 
    pivotGearbox, 0, Constants.IntakeConstants.armLength, 
    0, 60, false, 0, null);

  public IntakeIOSim(){
    pivotMotor
      .getConfigurator()
      .apply(new Slot0Configs().withKP(0.02).withKI(0.0).withKD(0.0).withKV(0.06));

      //find gear ratios
    spinMotor
      .getConfigurator()
      .apply(new Slot0Configs().withKP(0.02).withKI(0.0).withKD(0.0).withKV(0.06));


    this.intakeMotorSim = spinMotor.getSimState();
    this.pivotMotorSim= pivotMotor.getSimState();
    intakeMotorSim.setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);
    intakeMotorSim.setMotorType(MotorType.KrakenX60);
    pivotMotorSim.setMotorType(MotorType.KrakenX60);


  }

  public void updateInputs(IntakeIOInputs inputs){

    intakeMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    pivotMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

    armSim.update(0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    boolean isIntaking; 
    if (intakeMotorSim.getMotorVoltage() == 0){
      isIntaking = false;
    } else {
      isIntaking = true;
    } 

    inputs.isIntaking = isIntaking;
  }

  public void runIntake(Angle rotations){
    spinMotor.setControl(new VelocityVoltage(rotations.baseUnitMagnitude()));
  }

  public void pivotIntake(Angle angle){
    pivotMotor.setControl(new VelocityVoltage(angle.baseUnitMagnitude()));
  }
}
