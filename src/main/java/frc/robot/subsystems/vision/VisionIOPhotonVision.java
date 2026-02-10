package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.PoseObservation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final String name;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isCameraConnected = camera.isConnected();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

      Optional<MultiTargetPNPResult> maybeMultiTagResult = result.getMultiTagResult();
      if (maybeMultiTagResult.isPresent()) {
        MultiTargetPNPResult multiTagResult = maybeMultiTagResult.get();

        double totalTagDistance = 0.0;
        for (PhotonTrackedTarget target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        int tagCount = multiTagResult.fiducialIDsUsed.size();

        Transform3d bestTransform = multiTagResult.estimatedPose.best.plus(robotToCamera.inverse());
        Logger.recordOutput(
            "Vision/" + name + "EstimatedPose",
            new Pose3d(bestTransform.getTranslation(), bestTransform.getRotation()));

        inputs.poseObservation =
            new PoseObservation(
                new Pose3d(bestTransform.getTranslation(), bestTransform.getRotation()),
                result.getTimestampSeconds(),
                totalTagDistance / result.targets.size(),
                tagCount);
      }
    }
  }
}
