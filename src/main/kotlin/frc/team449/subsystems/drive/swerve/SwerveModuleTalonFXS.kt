package frc.team449.subsystems.drive.swerve

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.TalonFXS
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorPhaseValue
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.motorcontrol.Talon
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

@Logged
open class SwerveModuleTalonFXS(
  private val name: String,
  private val drivingMotor: TalonFX,
  private val turningMotor: TalonFXS,
  val turnController: PIDController,
  override val location: Translation2d
) : SwerveModule {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
    turnController.reset()
  }

  override val desiredState =
    SwerveModuleState(
      0.0,
      Rotation2d(),
    )

  /** The module's [SwerveModuleState], containing speed and angle. */
  override var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingMotor.velocity.valueAsDouble,
        Rotation2d(turningMotor.position.value),
      )
    }
    set(desState) {
      if (abs(desState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn more than 90 degrees. */
      desState.optimize(Rotation2d(turningMotor.position.value))

      desiredState.speedMetersPerSecond = desState.speedMetersPerSecond
      desiredState.angle = desState.angle
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  override val position: SwerveModulePosition
    get() {
      return SwerveModulePosition(
        drivingMotor.position.value.magnitude(),
        Rotation2d(turningMotor.position.value),
      )
    }

  override fun setVoltage(volts: Double) {
    desiredState.speedMetersPerSecond = 0.0
    turningMotor.setControl(
      PositionVoltage(turningMotor.position.value)
        .withEnableFOC(false)
        .withUpdateFreqHz(SwerveConstants.KRAKEN_UPDATE_RATE)
    )
    drivingMotor.setVoltage(volts)
  }

  /** Set module speed to zero but keep module angle the same. */
  override fun stop() {
    turningMotor.setControl(
      PositionVoltage(turningMotor.position.value)
        .withEnableFOC(false)
        .withUpdateFreqHz(SwerveConstants.KRAKEN_UPDATE_RATE)
    )
    drivingMotor.stopMotor()
    desiredState.speedMetersPerSecond = 0.0
  }

  override fun update() {
    /** CONTROL speed of module */
    drivingMotor.setControl(
      VelocityVoltage(desiredState.speedMetersPerSecond)
        .withUpdateFreqHz(SwerveConstants.KRAKEN_UPDATE_RATE)
        .withEnableFOC(false),
    )

    turningMotor.setControl(
      PositionVoltage(desiredState.angle.measure)
        .withEnableFOC(false)
        .withUpdateFreqHz(SwerveConstants.KRAKEN_UPDATE_RATE)
    )
  }

  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun createTalonFXSModule(
      name: String,
      driveID: Int,
      driveInverted: Boolean,
      turnID: Int,
      turnInverted: Boolean,
      turnEncoderChannel: Int,
      turnEncoderOffset: Double,
      turnEncoderInverted: Boolean,
      location: Translation2d
    ): SwerveModuleTalonFXS {
      val drivingMotor = TalonFX(driveID)

      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake
      config.MotorOutput.DutyCycleNeutralDeadband = SwerveConstants.DUTY_CYCLE_DEADBAND

      config.Feedback.SensorToMechanismRatio = 1 / (SwerveConstants.DRIVE_GEARING * SwerveConstants.DRIVE_UPR)

      config.Slot0.kP = SwerveConstants.DRIVE_KP
      config.Slot0.kI = SwerveConstants.DRIVE_KI
      config.Slot0.kD = SwerveConstants.DRIVE_KD
      config.Slot0.kS = SwerveConstants.DRIVE_KS
      config.Slot0.kV = SwerveConstants.DRIVE_KV
      config.Slot0.kA = SwerveConstants.DRIVE_KA

      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = SwerveConstants.DRIVE_STATOR_LIMIT.`in`(Amps)
      config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_SUPPLY_LIMIT.`in`(Amps)

      var status: StatusCode = StatusCode.StatusCodeNotInitialized
      for (i in 0..4) {
        status = drivingMotor.configurator.apply(config)
        if (status.isOK) break
      }
      if (!status.isOK) {
        println("Could not apply configs, error code: $status")
      }

      drivingMotor.statorCurrent.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.supplyCurrent.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.position.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.velocity.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.motorVoltage.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.closedLoopError.setUpdateFrequency(SwerveConstants.VALUE_UPDATE_RATE)
      drivingMotor.optimizeBusUtilization()


      val turnMotor = TalonFXS(turnID)

      val turnConfig = TalonFXSConfiguration()

      turnConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature
      turnConfig.ExternalFeedback.SensorPhase = SensorPhaseValue.Opposed
      turnConfig.ExternalFeedback.QuadratureEdgesPerRotation = 4096
      turnConfig.ExternalFeedback.AbsoluteSensorOffset = SwerveConstants.TURN_ENC_OFFSET_FL
      turnConfig.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 0.5
      turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true
      turnConfig.CurrentLimits.StatorCurrentLimitEnable = true
      turnConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.DRIVE_STATOR_LIMIT.`in`(Amps)
      turnConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_SUPPLY_LIMIT.`in`(Amps)

      turnConfig.Slot0.kP = SwerveConstants.TURN_KP
      turnConfig.Slot0.kI = SwerveConstants.TURN_KI
      turnConfig.Slot0.kD = SwerveConstants.TURN_KD
      turnConfig.Slot0.kS = SwerveConstants.STEER_KS

      turnConfig.ClosedLoopGeneral.ContinuousWrap = true

      turnMotor.configurator.apply(turnConfig)

      val turnController =
        PIDController(
          SwerveConstants.TURN_KP,
          SwerveConstants.TURN_KI,
          SwerveConstants.TURN_KD,
        )
      if (RobotBase.isReal()) {
        return SwerveModuleTalonFXS(
          name,
          drivingMotor,
          turnMotor,
          turnController,
          location,
        )
      } else {
        return SwerveModuleTalonFXS(
          name,
          drivingMotor,
          turnMotor,
          turnController,
          location,
        )
      }
    }
  }
}