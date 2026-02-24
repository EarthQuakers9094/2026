// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriverAutomations;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.KickerTemporaryCommand;
import frc.robot.commands.ShootFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOReal;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.EeshwarkTargeter;
import frc.robot.subsystems.shooter.targeter.Targeter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FieldUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;

  private final Vision vision;
  private final Targeter targeter = new EeshwarkTargeter();
  private final KickerSubsystem kicker;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private LinearFilter xInputAverage = LinearFilter.movingAverage(100);
  private LinearFilter yInputAverage = LinearFilter.movingAverage(100);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        shooter = new ShooterSubsystem(new ShooterIOReal());
        intake = new IntakeSubsystem(new IntakeIOReal());
        vision = new Vision(drive::addVisionMeasurement, new VisionIOPhotonVision("Left", null));
        kicker = new KickerSubsystem(new KickerIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        shooter = new ShooterSubsystem(new ShooterIOSim(drive::getPose, drive::getChassisSpeeds));
        intake = new IntakeSubsystem(new IntakeIOSim()); // fix

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "Front",
                    new Transform3d(0.2, 0.0, 0.5, new Rotation3d(0, -Math.PI / 7., 0.0)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Left",
                    new Transform3d(0.0, 0.2, 0.5, new Rotation3d(0, -Math.PI / 7., Math.PI / 2.)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Right",
                    new Transform3d(
                        0.0, -0.2, 0.5, new Rotation3d(0, -Math.PI / 7., -Math.PI / 2.)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Back",
                    new Transform3d(-0.2, 0.0, 0.5, new Rotation3d(0, -Math.PI / 7., Math.PI)),
                    drive::getPose));
        kicker = new KickerSubsystem(new KickerIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        vision = new Vision(drive::addVisionMeasurement);
        kicker = new KickerSubsystem(new KickerIO() {});
        break;
    }

    NamedCommands.registerCommand("shoot_fuel", new ShootFuel(shooter));
    NamedCommands.registerCommand("wait_for_spin_up", new WaitUntilCommand(shooter::isSpunUp));
    NamedCommands.registerCommand(
        "wait_for_eight_shot", new WaitUntilCommand(() -> shooter.shotCount >= 8));
    NamedCommands.registerCommand(
        "reset_shot_count", new InstantCommand(() -> shooter.shotCount = 0));

    NamedCommands.registerCommand("debug", Commands.print("Debug message from Pathplanner"));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    new Trigger(() -> FieldUtil.isNearTrench(drive.getPose()))
        .whileTrue(Commands.run(shooter::retractHood, shooter));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -1
                    * (shooter.isActivelyShooting()
                        ? yInputAverage.calculate(controller.getLeftY())
                        : controller.getLeftY()),
            () ->
                -1
                    * (shooter.isActivelyShooting()
                        ? xInputAverage.calculate(controller.getLeftX())
                        : controller.getLeftX()),
            () -> -controller.getRightX()));
    shooter.setDefaultCommand(
        DriverAutomations.targetHubOrFerry(
            shooter, drive::getPose, drive::getChassisSpeeds, targeter));
    // new ShooterTrackTarget(
    // shooter,
    // drive::getPose,
    // drive::getChassisSpeeds,
    // targeter,
    // Constants.Field.hubTarget,
    // true));

    controller.button(8).whileTrue(new ShootFuel(shooter));

    controller
        .button(7)
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d(Math.atan2(controller.getLeftX(), controller.getLeftY()))));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.button(9).toggleOnTrue(new IntakeFuel(intake));

    controller.y().toggleOnTrue(new KickerTemporaryCommand(kicker));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
