package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final VisionConsumer visionConsumer;

  public Vision(VisionConsumer visionConsumer, VisionIO... ios) {
    this.io = ios;
    this.inputs = new VisionIOInputsAutoLogged[ios.length];
    this.visionConsumer = visionConsumer;

    for (int i = 0; i < ios.length; i++) {
      this.inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      PoseObservation poseObservation = inputs[i].poseObservation;
      if (poseObservation != null) {
        // System.out.println("34" + poseObservation);

        double stdDevFactor =
            Math.pow(poseObservation.averageTagDistance(), 2)
                / poseObservation.averageTagDistance();

        double linearStdDev =
            Constants.Camera.linearStdDev
                * stdDevFactor
                * (poseObservation.tagCount == 1 ? 1000.0 : 1.0);
        double angularStdDev =
            Constants.Camera.angularStdDev
                * stdDevFactor
                * (poseObservation.tagCount == 1 ? 1000.0 : 1.0);

        visionConsumer.accept(
            poseObservation.pose.toPose2d(),
            poseObservation.timestamp,
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
      Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public record PoseObservation(
      Pose3d pose, double timestamp, double averageTagDistance, int tagCount) {}
}
