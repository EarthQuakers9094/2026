// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class SpindexerIOReal implements SpindexerIO {
  private final SparkFlex spindexerMotor =
      new SparkFlex(Constants.SpindexerConstants.spindexerMotorId, MotorType.kBrushless);
  private final SparkFlexConfig spindexerMotorConfig = new SparkFlexConfig();
  /** Creates a new SpindexerIOReal. */
  public SpindexerIOReal() {
    spindexerMotor.configure(
        spindexerMotorConfig
            .inverted(true)
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.SpindexerConstants.kP,
                        Constants.SpindexerConstants.kI,
                        Constants.SpindexerConstants.kD,
                        ClosedLoopSlot.kSlot0)
                    .pid(Constants.SpindexerConstants.kP * 0.0, 0, 0, ClosedLoopSlot.kSlot1)
                    .apply(new FeedForwardConfig().kV(0, ClosedLoopSlot.kSlot1))
                    .apply(
                        new FeedForwardConfig()
                            .kV(Constants.SpindexerConstants.kV, ClosedLoopSlot.kSlot0)))
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spindexerCurrentSpeed = RPM.of(spindexerMotor.getEncoder().getVelocity());
    inputs.spinexerVelocitySetpoint =
        RPM.of(spindexerMotor.getClosedLoopController().getSetpoint());

    inputs.spindexerCurrent = Amps.of(spindexerMotor.getOutputCurrent());
  }

  public void run(AngularVelocity spindexerSetSpeed) {
    if (Math.abs(spindexerSetSpeed.in(RPM)) < 100.0) {
      spindexerMotor
          .getClosedLoopController()
          .setSetpoint(spindexerSetSpeed.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    } else {
      spindexerMotor
          .getClosedLoopController()
          .setSetpoint(spindexerSetSpeed.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
  }
}
