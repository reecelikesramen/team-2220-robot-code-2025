// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Robot25;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.devices.DigitalInputWrapper;
import frc.robot.Robot;
import frc.robot.Robot25.commands.DriveCommands;
import frc.robot.Robot25.subsystems.drive.Drive;
import frc.robot.Robot25.subsystems.drive.DriveConstants;
import frc.robot.Robot25.subsystems.drive.ModuleIO;
import frc.robot.Robot25.subsystems.drive.ModuleIOSim;
import frc.robot.Robot25.subsystems.drive.ModuleIOTalonFX;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.ElevatorIO;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOSim;
import frc.robot.Robot25.subsystems.gyro.GyroIO;
import frc.robot.Robot25.subsystems.gyro.GyroIOPigeon2;
import frc.robot.Robot25.subsystems.gyro.GyroIOSim;
import frc.robot.Robot25.subsystems.outtake.Outtake;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOSim;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.SimConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends frc.lib.RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
      Drive.MAPLE_SIM_CONFIG,
      SimConstants.SIM_INITIAL_FIELD_POSE);

  // Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController OperatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // coast buttion
  private static DigitalInputWrapper coastButton = new DigitalInputWrapper(
      0,
      "coastButton",
      false);

  @AutoLogOutput
  public final Pose3d[] mechanismPoses = new Pose3d[] {
      Pose3d.kZero,
      Pose3d.kZero,
      Pose3d.kZero,
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super(driveSimulation);
    // Check for valid swerve config
    var modules = new SwerveModuleConstants[] {
        DriveConstants.FrontLeft,
        DriveConstants.FrontRight,
        DriveConstants.BackLeft,
        DriveConstants.BackRight,
    };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated ||
          constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.FrontLeft),
            new ModuleIOTalonFX(DriveConstants.FrontRight),
            new ModuleIOTalonFX(DriveConstants.BackLeft),
            new ModuleIOTalonFX(DriveConstants.BackRight));

        elevator = new Elevator(new ElevatorIO() {
        });
        outtake = new Outtake(new OuttakeIOTalonFX());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]));

        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });

        elevator = new Elevator(new ElevatorIO() {
        });

        outtake = new Outtake(new OuttakeIO() {
        });
        break;
    }

    NamedCommands.registerCommand("L1", elevator.L1());
    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand("L3", elevator.L3());
    NamedCommands.registerCommand("L4", elevator.L4());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>(
        "Auto Choices",
        AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Static Drive Voltage",
        Commands.run(() -> drive.driveOpenLoop(10)));
    autoChooser.addOption(
        "Static Turn Voltage",
        Commands.run(() -> drive.TurnOpenLoop(10)));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // toggle coast on true
    coastButton
        .asTrigger()
        .onChange(
            Commands.runOnce(() -> {
              // TODO Add coast for more subsystems once we have them
              drive.toggleCoast();
              System.out.println("COAST TOGGLED");
            }));

    // Xbox controller is mapped incorrectly on Mac OS
    DoubleSupplier xSupplier = () -> DriverController.getLeftX();
    DoubleSupplier ySupplier = () -> DriverController.getLeftY();
    DoubleSupplier omegaSupplier = () -> -DriverController.getRightX();
    BooleanSupplier slowModeSupplier = () -> !SimConstants.IS_MAC
        ? DriverController.getRightTriggerAxis() > 0.5
        : DriverController.getRightX() > 0.0;

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            ySupplier,
            xSupplier,
            omegaSupplier,
            slowModeSupplier));
    outtake.setDefaultCommand(outtake.autoQueueCoral());

    DriverController.a()
        .toggleOnTrue(
            DriveCommands.keepRotationForward(drive, xSupplier, ySupplier));

    // POV snap to angles
    DriverController.povUp()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.kZero));
    DriverController.povUpRight()
        .onTrue(
            DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-45)));
    DriverController.povRight()
        .onTrue(
            DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-90)));
    DriverController.povDownRight()
        .onTrue(
            DriveCommands.snapToRotation(
                drive,
                Rotation2d.fromDegrees(-135)));
    DriverController.povDown()
        .onTrue(
            DriveCommands.snapToRotation(
                drive,
                Rotation2d.fromDegrees(-180)));
    DriverController.povDownLeft()
        .onTrue(
            DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(135)));
    DriverController.povLeft()
        .onTrue(
            DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(90)));
    DriverController.povUpLeft()
        .onTrue(
            DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(45)));

    // TODO should this be true in TeleOp?
    // Switch to X pattern when X button is pressed
    DriverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when START button is pressed
    DriverController.start()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        Rotation2d.kZero)),
                drive).ignoringDisable(true));
    OperatorController.povDown().onTrue(elevator.minHeight());
    OperatorController.povUp().onTrue(elevator.maxHeight());

    OperatorController.a().onTrue(elevator.L1());
    OperatorController.x().onTrue(elevator.L2());
    OperatorController.b().onTrue(elevator.L3());
    OperatorController.y().onTrue(elevator.L4());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public Command getTestCommand() {
    return autoChooser.get();
  }

  @Override
  public void disabledInit() {
    // drive.stopWithX();
  }

  private ChassisSpeeds prevVelocity = new ChassisSpeeds();
  private double pitchVelocity = 0.0;
  private double pitch = 0.0;

  @Override
  public void simulationPeriodic() {
    var elevatorPoses = elevator.getElevatorPoses();
    mechanismPoses[0] = elevatorPoses[0];
    mechanismPoses[1] = elevatorPoses[1];
    mechanismPoses[2] = elevatorPoses[2];

    /* Derives velocity from drive simulation velocity and DT */
    var velocity = driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    var dVelocity = velocity.minus(prevVelocity);
    prevVelocity = velocity;
    var acceleration = dVelocity.div(0.02);
    Logger.recordOutput("AccelerationX", acceleration.vxMetersPerSecond);
    Logger.recordOutput("AccelerationY", acceleration.vyMetersPerSecond);

    /* Quadratic regression constants */
    final var a1 = 0.0010007;
    final var b1 = 0.101502;
    final var c1 = 7.85883;
    final var a2 = -0.0010289;
    final var b2 = 0.229311;
    final var c2 = 6.26015;

    /* Quadratic regression function */
    DoubleUnaryOperator estimateZCoM = h -> h < 0 ? 7.885
        : h < 45 ? a1 * h * h + b1 * h + c1 : h < 75 ? a2 * h * h + b2 * h + c2 : 17.731;

    // CoM Z in inches
    var zCoM = Inches.of(estimateZCoM.applyAsDouble(elevator.getExtension()));

    // 3D robot pose from 2D robot pose
    var robotPose3d = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());

    //
    var localRobotCoM = new Translation3d(Inches.of(-1.793095), Inches.of(0.824046), zCoM);

    // TODO +/- choose by direction
    var pitchPivot = new Translation3d(Inches.of(10.375), Inches.zero(), Inches.zero());

    if (pitch > 0 && pitch < Math.PI) {
      pitchPivot = pitchPivot.times(-1);
    }

    var toPitchPivot = new Transform3d(pitchPivot.times(-1), Rotation3d.kZero);
    var pitchRotation = new Transform3d(Translation3d.kZero, new Rotation3d(0, pitch, 0));
    var fromPitchPivot = new Transform3d(pitchPivot, Rotation3d.kZero);

    var worldRobotPose3d = robotPose3d.transformBy(toPitchPivot)
        .transformBy(pitchRotation).transformBy(fromPitchPivot);
    Logger.recordOutput("RobotPose3d", worldRobotPose3d);

    var worldRobotCoM = worldRobotPose3d
        .plus(new Transform3d(localRobotCoM, Rotation3d.kZero));
    Logger.recordOutput("RobotCoM", worldRobotCoM);

    final var M = DriveConstants.ROBOT_MASS_KG;
    final var G = MetersPerSecondPerSecond.of(9.81);
    final var PITCH_MOI = 5.042; // I_yy; kg m^2

    var externalTorque = M * -acceleration.vxMetersPerSecond * zCoM.in(Meters);
    var pivotToCoMX = worldRobotCoM.relativeTo(robotPose3d.plus(toPitchPivot)).getMeasureX();
    Logger.recordOutput("PivotToCoMX", pivotToCoMX);

    final var EQUILIBRIUM_THRESHOLD = Degrees.of(6).in(Radians) / 2;
    final var EFFECTIVE_PIVOT_SIGMOID_STEEPNESS = 5.0;

    var absPitch = pitch > Math.PI ? 2 * Math.PI - pitch : pitch;
    var effectivePivotSigmoidScalar = 1
        / (1 + Math.exp(-EFFECTIVE_PIVOT_SIGMOID_STEEPNESS * (absPitch / EQUILIBRIUM_THRESHOLD - 1)));
    var effectivePivotToCoMX = effectivePivotSigmoidScalar * pivotToCoMX.in(Meters);

    Logger.recordOutput("EffectivePivotToCoMX", effectivePivotToCoMX);

    final var PITCH_VELOCITY_DAMPING = 0;

    var gravityTorque = M * G.in(MetersPerSecondPerSecond) * effectivePivotToCoMX;
    var dampingTorque = -pitchVelocity * PITCH_VELOCITY_DAMPING;
    var pitchTorque = gravityTorque + externalTorque + dampingTorque;
    var pitchAccel = pitchTorque / PITCH_MOI;

    Logger.recordOutput("GravityTorque", gravityTorque);
    Logger.recordOutput("ExternalTorque", externalTorque);

    pitchVelocity += pitchAccel * 0.02;
    pitch += pitchVelocity * 0.02;
    pitch = pitch % (2 * Math.PI);

    if (pitch < 0)
      pitch = pitch + 2 * Math.PI;

    if (pitchVelocity > 0 && pitch > 0.5 * Math.PI && pitch < Math.PI) {
      pitch = 0.5 * Math.PI;
      pitchVelocity = 0;
    } else if (pitchVelocity < 0 && pitch < 1.5 * Math.PI && pitch > Math.PI) {
      pitch = 1.5 * Math.PI;
      pitchVelocity = 0;
    }

    final var GROUND_CONTACT_THRESHOLD = Degrees.of(7).in(Radians) / 2;
    final var GROUND_CONTACT_ENERGY_LOSS_SCALAR = 0.3;
    final var GROUND_CONTACT_SIGMOID_STEEPNESS = 8.0;
    var groundContactSigmoidScalar = 1
        / (1 + Math.exp(-GROUND_CONTACT_SIGMOID_STEEPNESS * (absPitch / GROUND_CONTACT_THRESHOLD - 1)));

    pitchVelocity *= groundContactSigmoidScalar * (1.0 -
        GROUND_CONTACT_ENERGY_LOSS_SCALAR)
        + GROUND_CONTACT_ENERGY_LOSS_SCALAR;

    Logger.recordOutput("PitchAcceleration", pitchAccel);
    Logger.recordOutput("PitchVelocity", pitchVelocity);
    Logger.recordOutput("Pitch", pitch);
  }
}
