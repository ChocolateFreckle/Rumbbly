package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generated.TunerConstants;
import frc.robot.Robot;
import frc.robot.SwerveModConstants;
import frc.robot.Conversions;

// Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
//             getCANcoder()
// Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
//             // getCANcoder()
//             Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()),

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(TunerConstants.driveGains.kS, TunerConstants.driveGains.kV, TunerConstants.driveGains.kA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModConstants moduleConstants){
      this.moduleNumber = moduleNumber;
      this.angleOffset = moduleConstants.angleOffset;
      
      /* Angle Encoder Config */
      angleEncoder = new CANcoder(moduleConstants.cancoderID);
      angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

      /* Angle Motor Config */
      mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
      mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
      resetToAbsolute();

      /* Drive Motor Config */
      mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
      mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
      mDriveMotor.getConfigurator().setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
      desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
      mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
      setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
      if(isOpenLoop){
          driveDutyCycle.Output = desiredState.speedMetersPerSecond / TunerConstants.Swerve.maxSpeed;
          mDriveMotor.setControl(driveDutyCycle);
      }
      else {
          driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, TunerConstants.Swerve.wheelCircumference);
          driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
          mDriveMotor.setControl(driveVelocity);
      }
  }

  public Rotation2d getCANcoder(){
      return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute(){
      double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
      mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState(){
      return new SwerveModuleState(
          Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), TunerConstants.Swerve.wheelCircumference), 
          Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
      );
  }

  public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(
          Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), TunerConstants.Swerve.wheelCircumference), 
          Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
      );
  }
}