// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DispenseFuel;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriverAutomations;
import frc.robot.commands.EjectLooseFuel;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.ManualTurret;
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
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
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
  private final SpindexerSubsystem spindexer;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick leftStick = new CommandJoystick(1);
  private final CommandJoystick rightStick = new CommandJoystick(2);

  private LinearFilter xInputAverage = LinearFilter.movingAverage(15);
  private LinearFilter yInputAverage = LinearFilter.movingAverage(15);

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
        shooter = new ShooterSubsystem(new ShooterIOReal(), drive::getPose);
        intake = new IntakeSubsystem(new IntakeIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    "Front",
                    new Transform3d(
                        Inches.of(12.465),
                        Inches.of(4.915),
                        Inches.of(12.03),
                        new Rotation3d(0, -Math.PI / 6., 0.0))),
                new VisionIOPhotonVision(
                    "Front",
                    new Transform3d(0.320, 0.163, 0.210, new Rotation3d(0, -Math.PI / 12., 0))),
                new VisionIOPhotonVision(
                    "Left",
                    new Transform3d(
                        0.247, 0.345, 0.277, new Rotation3d(0, -Math.PI / 8., Math.PI / 2.))),
                new VisionIOPhotonVision(
                    "Right",
                    new Transform3d(
                        0.226, -0.345, 0.277, new Rotation3d(0, -Math.PI / 8., -Math.PI / 2.))));
        kicker = new KickerSubsystem(new KickerIOReal());
        spindexer = new SpindexerSubsystem(new SpindexerIOReal());
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
        shooter =
            new ShooterSubsystem(
                new ShooterIOSim(drive::getPose, drive::getChassisSpeeds), drive::getPose);
        intake = new IntakeSubsystem(new IntakeIOSim()); // fix
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "Front",
                    new Transform3d(
                        Inches.of(12.465),
                        Inches.of(4.915),
                        Inches.of(12.03),
                        new Rotation3d(0, -Math.PI / 6., 0.0)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Front",
                    new Transform3d(0.320, 0.163, 0.210, new Rotation3d(0, -Math.PI / 12., 0)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Left",
                    new Transform3d(
                        0.247, 0.345, 0.277, new Rotation3d(0, -Math.PI / 8., Math.PI / 2.)),
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    "Right",
                    new Transform3d(
                        0.226, -0.345, 0.277, new Rotation3d(0, -Math.PI / 8., -Math.PI / 2.)),
                    drive::getPose));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             "Front",
        //             new Transform3d(0.2, 0.0, 0.5, new Rotation3d(0, -Math.PI / 7., 0.0)),
        //             drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             "Left",
        //             new Transform3d(0.0, 0.2, 0.5, new Rotation3d(0, -Math.PI / 7., Math.PI /
        // 2.)),
        //             drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             "Right",
        //             new Transform3d(
        //                 0.0, -0.2, 0.5, new Rotation3d(0, -Math.PI / 7., -Math.PI / 2.)),
        //             drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             "Back",
        //             new Transform3d(-0.2, 0.0, 0.5, new Rotation3d(0, -Math.PI / 7., Math.PI)),
        //             drive::getPose));
        kicker = new KickerSubsystem(new KickerIOSim());
        spindexer = new SpindexerSubsystem(new SpindexerIOSim());
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
        shooter = new ShooterSubsystem(new ShooterIO() {}, drive::getPose);
        intake = new IntakeSubsystem(new IntakeIO() {});
        // vision = null;
        vision = new Vision(drive::addVisionMeasurement);
        kicker = new KickerSubsystem(new KickerIO() {});
        spindexer = new SpindexerSubsystem(new SpindexerIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "deploy_intake", new InstantCommand(() -> intake.deployIntake(), intake));
    NamedCommands.registerCommand(
        "retract_intake", new InstantCommand(() -> intake.retractIntake(), intake));
    NamedCommands.registerCommand(
        "jiggle_intake",
        Commands.sequence(
            new InstantCommand(() -> intake.retractIntake()),
            new WaitCommand(0.5),
            new InstantCommand(() -> intake.deployIntake())));

    NamedCommands.registerCommand("start_intake", new InstantCommand(() -> intake.startIntake()));
    NamedCommands.registerCommand("stop_intake", new InstantCommand(() -> intake.stopIntake()));
    NamedCommands.registerCommand("eject_loose_fuel", new EjectLooseFuel(shooter));

    NamedCommands.registerCommand(
        "retract_hood",
        Commands.run(
            () -> {
              shooter.retractHood();
              //   System.out.println("retract hood");
            },
            shooter));

    NamedCommands.registerCommand("shoot_fuel", new ShootFuel(shooter, kicker, intake, true));
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

    new Trigger(shooter::isActivelyShooting)
        .onTrue(new InstantCommand(spindexer::start).ignoringDisable(true))
        .onFalse(new InstantCommand(spindexer::stop).ignoringDisable(true));

    new Trigger(() -> FieldUtil.isNearTrench(drive.getPose()))
        .whileTrue(Commands.run(shooter::retractHood).ignoringDisable(true));

    // new Trigger(() -> FieldUtil.inAllianceZone(drive.getPose(),
    // DriverStation.getAlliance().orElse(Alliance.Blue)))
    //         .whileTrue(new RevvShooter(shooter, kicker));

    // Configure the button bindings
    if (!Constants.debugMode) {
      configureButtonBindings();
    } else {
      configureTestingBindings();
    }
  }

  private void configureTestingBindings() {
    leftStick
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.setPitch(Rotation2d.fromDegrees(90))));
    leftStick
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.setPitch(Rotation2d.fromDegrees(0))));
    leftStick
        .povRight()
        .onTrue(new InstantCommand(() -> shooter.setPitch(Rotation2d.fromDegrees((80 + 55) / 2))));
    // leftStick.povUp().onTrue(new InstantCommand(() ->
    // shooter.setYaw(Rotation2d.fromDegrees(0))));
    // leftStick
    //     .povLeft()
    //     .onTrue(new InstantCommand(() -> shooter.setYaw(Rotation2d.fromDegrees(45))));
    // leftStick
    //     .povRight()
    //     .onTrue(new InstantCommand(() -> shooter.setYaw(Rotation2d.fromDegrees(-45))));
    // // leftStick.button(2).whileTrue(new ShootFuel(shooter, kicker));
    // SmartDashboard.putNumber("HoodAngle", 0);
    // leftStick
    //     .trigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setPitch(new Rotation2d(SmartDashboard.getNumber("HoodAngle", 90)));
    //             }));

    rightStick.trigger().toggleOnTrue(new DeployIntake(intake));

    // leftStick.povUp().onTrue(new Inst)
    // controller.x().onTrue(new KickerShooterSpindexerCommand(kicker, shooter, spindexer));
    // controller
    //     .y()
    //     .onTrue(
    //         Commands.parallel(
    //             new InstantCommand(() -> shooter.endShooting(), shooter),
    //             new InstantCommand(() -> kicker.stopKicker(), kicker),
    //             new InstantCommand(() -> spindexer.stop(), spindexer)));

    // TODO make this conditional command work by making a boolean condition so that when
    // controller.a is triggered it will decide between running and stopping...
    /*controller.a().onTrue
    (
        new ConditionalCommand
        (
            new KickerShooterSpindexerCommand(kicker, shooter, spindexer),
            Commands.parallel
            (
                new InstantCommand(() -> shooter.endShooting()),
                new InstantCommand(() -> kicker.stopKicker()),
                new InstantCommand(() -> spindexer.stop())
            ),
            () -> (true)
        )
    );*/
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> {
              double y = (shooter.isActivelyShooting() ? 0.7 : 1.0) * leftStick.getY();
              double smoothedY = yInputAverage.calculate(y);
              return -1 * (shooter.isActivelyShooting() ? smoothedY : y);
            },
            () -> {
              double x = (shooter.isActivelyShooting() ? 0.7 : 1.0) * leftStick.getX();
              double smoothedX = xInputAverage.calculate(x);
              return -1 * (shooter.isActivelyShooting() ? smoothedX : x);
            },
            () -> -(shooter.isActivelyShooting() ? 0.5 * rightStick.getX() : rightStick.getX())));
    shooter.setDefaultCommand(
        DriverAutomations.targetHubOrFerry(
                shooter, drive::getPose, drive::getChassisSpeeds, targeter)
            .onlyIf(() -> !FieldUtil.isNearTrench(drive.getPose())));
    // new ShooterTrackTarget(
    // shooter,
    // drive::getPose,
    // drive::getChassisSpeeds,
    // targeter,
    // Constants.Field.hubTarget,
    // true));

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

    // controller.button(9).toggleOnTrue(new IntakeFuel(intake));

    // controller.y().toggleOnTrue(new KickerTemporaryCommand(kicker));

    // controller.x().toggleOnTrue(new SpindexerCommand(spindexer));

    /*
     * left trigger intrake fuel -- hold
     * right trigger deploy intake -- hold
     * right tirigger left face button deploy intake -- toggle TODO
     * left trigger right face button toggle drive mode
     */

    rightStick.trigger().whileFalse(new DeployIntake(intake));
    leftStick.trigger().whileTrue(new IntakeFuel(intake));
    leftStick.button(2).whileTrue(new ShootFuel(shooter, kicker, intake));
    rightStick.button(2).whileTrue(new DispenseFuel(intake));

    leftStick
        .button(4)
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -rightStick.getY(),
                () -> -rightStick.getX(),
                () -> new Rotation2d(Math.atan2(rightStick.getX(), rightStick.getY()))));

    leftStick
        .button(3)
        .onTrue(
            new InstantCommand(
                () -> intake.setIntakePosition(Constants.IntakeConstants.deployedAngle)));

    controller
        .start()
        .whileTrue(
             new ManualTurret(shooter, controller::getLeftX));

    controller.leftTrigger().whileTrue(Commands.run(shooter::retractHood, shooter));
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
