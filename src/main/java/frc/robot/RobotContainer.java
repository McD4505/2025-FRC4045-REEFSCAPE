// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldUtil.ReefSide;
import frc.robot.autos.BargeWingAuto;
import frc.robot.commands.DriveToTargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator.ReefLevel;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> m_Chooser = AutoBuilder.buildAutoChooser();

    Vision vision = new Vision();

    private Elevator elevator = new Elevator();

    private Dispenser dispenser = elevator.getDispenser();

    CommandXboxController controller2 = new CommandXboxController(1);

    public RobotContainer() {

        configureBindings();

        SmartDashboard.putData("auto chooser", m_Chooser);

        m_Chooser.addOption("target, pathfind, score", 
                new InstantCommand(() -> Vision.targetBranch(drivetrain, false))
                .andThen(new DriveToTargetPose(drivetrain))
                .andThen(elevator.score(ReefLevel.LEVEL_4))
        );

        Supplier<Boolean> isRedSupplier = () -> {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        };

        Supplier<Command> bargeAutoCommand = () -> {
            return new BargeWingAuto(drivetrain, elevator, dispenser, isRedSupplier.get(), false);
        };

        Supplier<Command> otherBargeAutoCommand = () -> {
            return new BargeWingAuto(drivetrain, elevator, dispenser, isRedSupplier.get(), true);
        };

        m_Chooser.addOption("barge wing", 
            Commands.defer(
                bargeAutoCommand, Set.of(drivetrain, elevator, dispenser)
            )
        );

        m_Chooser.addOption("other barge wing", 
            Commands.defer(
                otherBargeAutoCommand, Set.of(drivetrain, elevator, dispenser)
            )
        );
    }

    private Command pathfindToSide(ReefSide side) {
        Supplier<Boolean> isRedSupplier = () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        // stations
        if(side == ReefSide.LEFT_STATION || side == ReefSide.RIGHT_STATION) {
            return Commands.defer(() -> {
                return drivetrain.pathfindToPose(FieldUtil.getStationPose(side, isRedSupplier.get()));
            }, Set.of(drivetrain));
        }
        // reef sides
        return Commands.defer(() -> {
            return drivetrain.pathfindToPose(FieldUtil.getPreScoringPose(side, isRedSupplier.get()));
        }, Set.of(drivetrain));
    }

    private void configureSecondController() {
        controller2.leftBumper().onTrue(elevator.score(ReefLevel.LEVEL_4));

        Command addMeasurement1 = new InstantCommand(() -> {
            Vision.addVisionMeasurementMT1(drivetrain, "limelight");
        }).repeatedly();
        Command addMeasurement2 = new InstantCommand(() -> {
            Vision.addVisionMeasurementMT1(drivetrain, "limelight-two");
        }).repeatedly();

        controller2.rightTrigger().whileTrue(addMeasurement1.alongWith(addMeasurement2));
        controller2.leftTrigger().onTrue(dispenser.setSpeedCommand(1))
        .onFalse(dispenser.stopCommand());
        

        Trigger closeTrigger = controller2.a();
        Trigger farTrigger = controller2.y();
        Trigger leftCloseTrigger = controller2.x();
        Trigger rightCloseTrigger = controller2.b();
        Trigger leftFarTrigger = controller2.x().and(controller2.rightBumper());
        Trigger rightFarTrigger = controller2.b().and(controller2.rightBumper());

        Trigger leftStationTrigger = controller2.povLeft();
        Trigger rightStationTrigger = controller2.povRight();

        closeTrigger.onTrue(pathfindToSide(ReefSide.CLOSE));  // close
        farTrigger.onTrue(pathfindToSide(ReefSide.FAR));  // far
        leftCloseTrigger.onTrue(pathfindToSide(ReefSide.LEFT_CLOSE));  // left close
        rightCloseTrigger.onTrue(pathfindToSide(ReefSide.RIGHT_CLOSE));  // right close

        leftFarTrigger.onTrue(pathfindToSide(ReefSide.LEFT_FAR));  // left far
        rightFarTrigger.onTrue(pathfindToSide(ReefSide.RIGHT_FAR));  // right far

        leftStationTrigger.onTrue(pathfindToSide(ReefSide.LEFT_STATION));  // left station
        rightStationTrigger.onTrue(pathfindToSide(ReefSide.RIGHT_STATION));  // right station
    }

    private void configureArbitraryTriggers() {
        // disable elevator PID trigger
        Trigger disableElevatorPIDTrigger = new Trigger(elevator::shouldDisablePID);
        disableElevatorPIDTrigger.onTrue(elevator.disablePIDCommand().ignoringDisable(true));
        disableElevatorPIDTrigger.onChange(new PrintCommand("disable elevator PID changed"));

        // zero angle motor trigger
        Trigger zeroAngleMotorTrigger = new Trigger(() -> 
            dispenser.isLimitSwitchPressed() && 
            (elevator.getLevel() == ReefLevel.BASE || elevator.getLevel() == ReefLevel.STOWED));
        zeroAngleMotorTrigger.onTrue(dispenser.zeroAngleMotorCommand().ignoringDisable(true));
        
        // LED intake trigger
        Trigger ledIntakeTrigger = new Trigger(elevator::hasCoralChanged);
        ledIntakeTrigger.onTrue(elevator.updateLEDIntakeStateCommand().ignoringDisable(true));
        
        // push coral trigger
        Trigger pushInCoralTrigger = new Trigger(dispenser::hasCoral);
        pushInCoralTrigger.onTrue(dispenser.turnWheelSlightlyCommand());
    }

    private void configureBindings() {
        configureSecondController();
        configureArbitraryTriggers();
        
        // target left/right branch
        joystick.povLeft().onTrue(new InstantCommand(() -> Vision.targetBranch(drivetrain, true)));
        joystick.povRight().onTrue(new InstantCommand(() -> Vision.targetBranch(drivetrain, false)));

        joystick.povDown().onTrue(new InstantCommand(() -> Vision.targetStation(drivetrain)));
        
        // pathfind button
        joystick.povUp().onTrue(new DriveToTargetPose(drivetrain));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * elevator.getMaxSpeedFactor()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * elevator.getMaxSpeedFactor()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * elevator.getMaxSpeedFactor()) // Drive counterclockwise with negative X (left)
            )
        );

        // raise elevator bindings
        joystick.a().onTrue(elevator.setTargetCommand(ReefLevel.LEVEL_3));
        joystick.b().onTrue(elevator.setTargetCommand(ReefLevel.LEVEL_4));
        joystick.x().onTrue(elevator.setTargetCommand(ReefLevel.BASE));
        joystick.y().onTrue(elevator.setTargetCommand(ReefLevel.INTAKE));
        joystick.start().and(joystick.a()).onTrue(elevator.setTargetCommand(ReefLevel.LEVEL_2));

        // other elevator bindings
        joystick.back().and(joystick.a()).onTrue(elevator.setTargetCommand(ReefLevel.STOWED));
        joystick.back().and(joystick.x()).onTrue(elevator.setTargetCommand(ReefLevel.DISABLED));
        joystick.back().and(joystick.b()).onTrue(elevator.setTargetCommand(ReefLevel.HIGH));

        // dispenser bindings
        joystick.rightBumper().onTrue(dispenser.setSpeedCommand(3.5))
        .onFalse(dispenser.stopCommand());

        joystick.rightTrigger().onTrue(dispenser.setSpeedCommand(-3.5))
        .onFalse(dispenser.stopCommand());

        joystick.leftTrigger().onTrue(elevator.resetLiftCommand());

        joystick.leftStick().onTrue(drivetrain.getDefaultCommand());  // interrupt drivetrain command to cancel pathfinding
        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        .andThen(drivetrain.recalculateOperatorPerspectiveCommand()));

        drivetrain.registerTelemetry(logger::telemeterize);        
    }

    public Command getAutonomousCommand() {
        return m_Chooser.getSelected();
    }
}
