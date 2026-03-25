// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoveIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.vision.VisionIO;
// import frc.robot.subsystems.vision.VisionIOLimelight;
// import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  //   private Vision vision;
  public static shooter shooter = new shooter();
  public static intake intake = new intake();

  // Controller
  private final CommandPS4Controller controller = new CommandPS4Controller(0);
  private final CommandPS4Controller fuelMinipulator = new CommandPS4Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Ramp rate limiters (acceleration)
  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); // m/s^2
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0); // rad/s^2
  private final Vision vision;

  double xSpeed = xLimiter.calculate(controller.getLeftX());
  double ySpeed = yLimiter.calculate(controller.getLeftY());
  double rot = rotLimiter.calculate(controller.getRightX());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:

        // new VisionIOLimelight(camera1Name, drive::getRotation));

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
        // Real robot, instantiate hardware IO implementations
        // vision =
        // new Vision(
        // demoDrive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0),
        // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-first", drive::getRotation));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // Sim robot, instantiate physics sim IO implementations
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //
        // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim("limelight-first", new Transform3d(), drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Create Named Commands
    NamedCommands.registerCommand(
        "Reset Gyro",
        Commands.runOnce(
                () ->
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
                drive)
            .ignoringDisable(true));
    NamedCommands.registerCommand(
        "Shoot",
        new SequentialCommandGroup(shooter.setConvayerbelt(0.68), shooter.setTransition(0.68)));
    NamedCommands.registerCommand("StartIntake", intake.setSpeed(1));
    NamedCommands.registerCommand("StopIntake", intake.setSpeed(0));
    NamedCommands.registerCommand("LowerIntake", intake.setIntakePosition(0));
    NamedCommands.registerCommand("HigherIntake", intake.setIntakePosition(0.695));
    NamedCommands.registerCommand("StartShooter", shooter.setShooterSpeed(0.62));
    NamedCommands.registerCommand("Transition", shooter.setTransition(0.68));
    NamedCommands.registerCommand("ConveyorBelt", shooter.setConvayerbelt(0.68));
    NamedCommands.registerCommand(
        "AutoAim",
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> drive.getAngleToHub()));

    // Auto Command inits
    PathPlannerAuto autoCommand = new PathPlannerAuto("Blue 1");

    // Bind different auto triggers
    autoCommand.isRunning().onTrue(Commands.print("Auto Started"));

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
    autoChooser.addOption("Blue 1 Fuel&Shoot", autoCommand);

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to Center of Hub when X button is held
    controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> drive.getAngleToHub()));

    // Lock to semicircle around hub
    // controller
    //   .square()
    // .whileTrue(
    //   DriveCommands.joystickDriveAtAngle(
    //        drive,
    //        () -> -drive.getDistanceYToHub(),
    //       () -> -drive.getDistanceXToHub(),
    //       () -> Rotation2d.kZero));
    // Switch to X pattern when Triangle button is pressed
    controller.triangle().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Circle button is pressed
    controller
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
                    drive)
                .ignoringDisable(true));

    // Intake when circle button pressed
    fuelMinipulator.circle().whileTrue(intake.setSpeed(1));
    fuelMinipulator.circle().whileFalse(intake.setSpeed(0));

    // Shooter when l2 pressed
    fuelMinipulator.L2().whileTrue(shooter.setShooterSpeed(0.62));
    fuelMinipulator.L2().whileFalse(shooter.setShooterSpeed(0));

    // Transition when L1 pressed
    fuelMinipulator.L1().whileTrue(shooter.setTransition(0.68));
    fuelMinipulator.L1().whileTrue(shooter.setConvayerbelt(0.68));
    fuelMinipulator.L1().whileFalse(shooter.setTransition(0));
    fuelMinipulator.L1().whileFalse(shooter.setConvayerbelt(0));
    fuelMinipulator.R1().whileTrue(shooter.setTransition(-0.68));
    fuelMinipulator.R1().whileTrue(shooter.setConvayerbelt(-0.68));
    fuelMinipulator.R1().whileFalse(shooter.setTransition(0));
    fuelMinipulator.R1().whileFalse(shooter.setConvayerbelt(0));

    // Auto aim command example
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .square()
    //     .onTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));
    fuelMinipulator.R2().onTrue(new MoveIntake(.695));
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
