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

// import com.google.flatbuffers.Constants;

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
      Transform3d bestTransform = null;
      double totalTagDistance = 0;
      int tagCount = 0;
      if (maybeMultiTagResult.isPresent()) {
        MultiTargetPNPResult multiTagResult = maybeMultiTagResult.get();

        // double totalTagDistance = 0.0;
        for (PhotonTrackedTarget target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        tagCount = multiTagResult.fiducialIDsUsed.size();

        bestTransform = multiTagResult.estimatedPose.best.plus(robotToCamera.inverse());
      } /*else {
          if (!result.targets.isEmpty()) {
            PhotonTrackedTarget target = result.targets.get(0);

            var tagPose = Constants.Field.aprilTagLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              Transform3d fieldToTarget =
                  new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
              Transform3d cameraToTarget = target.getBestCameraToTarget();
              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              bestTransform = fieldToCamera.plus(robotToCamera.inverse());
            }
            // Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
            // fieldToRobot.getRotation());
            // bestTransform =
            // target.getBestCameraToTarget()//.bestCameraToTarget.plus(robotToCamera.);
            tagCount = 1;
          }
        }*/

      if (bestTransform != null) {
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
