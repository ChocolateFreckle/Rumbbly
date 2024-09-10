// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  public Arm arm = new Arm();
  public LaserCan lc;
  public TalonFX LauncherFeedMotor = new TalonFX(TunerConstants.LaunchFeed);
  public TalonFX IntakeFeedMotor = new TalonFX(TunerConstants.IntakeFeed);
  public TalonFX IntakeCenterMotor = new TalonFX(TunerConstants.IntakeCenter);
  public TalonFX LaunchRtFlywheel = new TalonFX(TunerConstants.LaunchRtFlywheel);
  public TalonFX LaunchLtFlywheel = new TalonFX(TunerConstants.LaunchLtFlywheel);
  public LaserCan ArmAngleSensor = new LaserCan(TunerConstants.ArmSensor);
  public LaserCan IntakeSensor = new LaserCan(TunerConstants.IntakeSensor);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My drive joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final CommandXboxController opstick = new CommandXboxController(1); //My operator joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    opstick.button(8).onTrue(new InstantCommand(()->{
          arm.enable();
          arm.setGoal(setpoint1);
          LaunchRtFlywheel.set(warmup);
          LaunchLtFlywheel.set(warmup);
          }));

    opstick.button(1).onTrue(new InstantCommand(()->{
      arm.setGoal(setpoint1);
      IntakeFeedMotor.set(fullSpeedAhead);
      IntakeCenterMotor.set(fullSpeedAhead);
      LauncherFeedMotor.set(fullSpeedAhead);
      shootOff();
      }));

    opstick.button(2)
    .onTrue(new InstantCommand(()->arm.setGoal(setpoint2)));

    opstick.button(3)
    .onTrue(new InstantCommand(()->arm.setGoal(setpoint3)));

    opstick.button(4)
          .onTrue(new InstantCommand(()->{
            LaunchRtFlywheel.set(fullSpeedAhead);
            LaunchLtFlywheel.set(fullSpeedAhead);
            LauncherFeedMotor.set(fullSpeedAhead);
          })); 
  }

    double setpoint1 = 05;
    double setpoint2 = 30;
    double setpoint3 = 60;
    double fullSpeedAhead = 1;
    double Die = 0;
    double warmup = 0.5;

  public RobotContainer() {
    configureBindings();
  }

  public void shootOff(){
    IntakeSensor.getMeasurement();
    if (LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT<=100){
        IntakeFeedMotor.set(Die);
        IntakeCenterMotor.set(Die);
        LauncherFeedMotor.set(Die);
        }        
  }

  public void checkSetpoint(){
    if (arm.=setpoint1){
  
        }        
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}