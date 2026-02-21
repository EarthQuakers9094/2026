package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MovingAverage;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

        private final ShooterIO io;
        private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
        private final Supplier<Translation2d> robotPositionSupplier;
        private final BooleanSupplier shouldTargetHub;
        private final MovingAverage speedAverage = new MovingAverage(40);

        @AutoLogOutput
        private boolean isShooting = false;
        @AutoLogOutput
        private boolean isIndexing = false;

        public ShooterSubsystem(
                        ShooterIO io, Supplier<Pose2d> robotPositionSupplier, BooleanSupplier shouldTargetHub) {
                this.io = io;
                this.robotPositionSupplier = () -> robotPositionSupplier.get().getTranslation();
                this.shouldTargetHub = shouldTargetHub;
        }

        public void beginShooting() {
                isShooting = true;
                io.setVelocitySetpoint(Constants.ShooterConstants.launchSpeed);
                Logger.recordOutput(
                                "Shooter/SpeedSetpointRadPerSec",
                                Constants.ShooterConstants.launchSpeed.in(RadiansPerSecond));
        }

        public void endShooting() {
                isShooting = false;
                io.setVelocitySetpoint(RPM.of(0.0));
                Logger.recordOutput("Shooter/SpeedSetpointRadPerSec", 0.0);
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);

                if (shouldTargetHub.getAsBoolean()) {
                        Translation3d hubTarget = Constants.Field.hubTarget;
                        Translation2d shooter = robotPositionSupplier
                                        .get()
                                        .plus(Constants.ShooterConstants.positionOnRobot.toTranslation2d());

                        double x = hubTarget.toTranslation2d().getDistance(shooter); // target distance in meters
                        double y = hubTarget.getZ() - Constants.ShooterConstants.positionOnRobot.getZ(); // target
                        // height in
                        // meters
                        //

                        double g = 9.81;
                        double v = inputs.shooterSpeed.in(RadiansPerSecond)
                                        * Constants.ShooterConstants.flywheelDiameter.in(Meters);
                        double v_squared = Math.pow(v, 2);

                        // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
                        double pitchRadians = Math.atan2(
                                        v_squared
                                                        + Math.sqrt(
                                                                        Math.pow(v_squared, 2) - g * (g * Math.pow(x, 2)
                                                                                        + 2 * y * v_squared)),
                                        g * x);

                        double timeOfFlightSeconds = x / (v * Math.cos(pitchRadians));

                        Logger.recordOutput("Shooter/TargetingHeight", y);
                        Logger.recordOutput("Shooter/TargetingVelocity", v);
                        Logger.recordOutput("Shooter/TargetingDistance", x);
                        Logger.recordOutput("Shooter/TargetingPitch", pitchRadians);

                        io.setPitch(new Rotation2d(pitchRadians));
                }
                double averageSpeed = speedAverage.addValue(inputs.shooterSpeed.baseUnitMagnitude());

                Logger.recordOutput("Shooter/AverageSpeed", averageSpeed);
                if (isShooting
                                && averageSpeed >= Constants.ShooterConstants.minLaunchSpeed.baseUnitMagnitude()) {
                        io.startIndexing();
                        isIndexing = true;

                } else {
                        io.stopIndexing();
                        isIndexing = false;
                }
                Logger.processInputs("Shooter", inputs);
        }
}
