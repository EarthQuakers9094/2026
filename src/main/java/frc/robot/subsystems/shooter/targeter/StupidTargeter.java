// package frc.robot.subsystems.shooter.targeter;

// import java.util.Optional;
// import java.util.function.Supplier;

// import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;

// public class StupidTargeter implements KinematicTargeter {

//     public enum StupidTargetingPosition {
//         LeftCorner,
//         RightCorner,
//         Hub
//     }

//     private final Supplier<StupidTargetingPosition> targetingPositionSupplier;

//     public StupidTargeter(Supplier<StupidTargetingPosition> targetingPositionSupplier) {
//         this.targetingPositionSupplier = targetingPositionSupplier;
//     }

//     @Override
//     public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
//         switch (targetingPositionSupplier.get()) {
//             case LeftCorner:

//                 break;
//             case RightCorner:
//                 break;
//             case Hub:
//                 break;
//             default:
//                 break;
//         }

//         this.getShooterTargetingWithoutVelocity(0, 0, 0)
//     }

// }
