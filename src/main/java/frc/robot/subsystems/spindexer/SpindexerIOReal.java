// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerIOReal implements SpindexerIO {
  private DCMotor spindexeDcMotor = DCMotor.getNeoVortex(1);
  private final SparkFlex spindexerMotor = new SparkFlex(
    Constants.SpindexerConstants.spindexerMotorId, MotorType.kBrushless);
  private final SparkFlexConfig spindexerMotorConfig = new SparkFlexConfig();
  /** Creates a new SpindexerIOReal. */
  public SpindexerIOReal() {
    spindexerMotor.configure(spindexerMotorConfig.apply
        (new ClosedLoopConfig().pid(0.1, 0, 0.1)), 
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void run(AngularVelocity spindexerSetSpeed){
        spindexerMotor.getClosedLoopController().setSetpoint(spindexerSetSpeed.in(RPM), ControlType.kVelocity);
    }
}
