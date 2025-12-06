package frc.team449.subsystems.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax

@Logged
class Intake(
  private val intakeMotor: TalonFX,
  private val funnelerMotor: SparkMax,
  follower2: SparkMax,
  private val indexerMotor: SparkMax,
  follower3: SparkMax,
  private val conveyorMotor: SparkMax,
  private val shooterMotor: TalonFX,
  private val rightSensor: LaserCanInterface,
  private val leftSensor: LaserCanInterface,
  private val shooterSensor: LaserCanInterface
  ): SubsystemBase() {
  private val timer = Timer()
  private val shootingDebouncer = Debouncer(IntakeConstants.SHOOTING_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling)
//
//  private val sensors =
//    listOf(
//      leftSensor,
//      rightSensor,
//      shooterSensor,
//    )
//
//  private var allSensorsConfigured = true
//  private var intakingSensorDown = false
//  private var shootingSensorDown = false
//  private var lasercanConfigured = arrayOf<Boolean>()
//
//  private fun configureSensors() {
//    allSensorsConfigured = true
//    intakingSensorDown = false
//    shootingSensorDown = false
//    lasercanConfigured = arrayOf<Boolean>()
//    try {
//      for (sensor in sensors) {
//        sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS)
//        sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
//        sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
//        lasercanConfigured += true
//      }
//    } catch (_: Exception) {
//      lasercanConfigured += false
//      allSensorsConfigured = false
//    }
//    if((lasercanConfigured.size < 3) || !lasercanConfigured[2]) {
//      shootingSensorDown = true
//    }
//    if((lasercanConfigured.size < 2) || !(lasercanConfigured[0] && lasercanConfigured[1])) { //DEMORGANSSSS
//      intakingSensorDown = true
//    }
//    timer.reset()
//  }

  init {
//    configureSensors()
    timer.start()
  }

  private fun detectsPiece(sensor: LaserCanInterface): Boolean {
    val measurement = sensor.measurement
    return measurement != null && (
      measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        measurement.distance_mm <= IntakeConstants.CORAL_DETECTION_THRESHOLD
      )
  }

  private fun pieceIntaken(): Boolean {
    return detectsPiece(rightSensor) || detectsPiece(leftSensor)
  }

  fun intake(): Command {
    return Commands.sequence(
      runOnce {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
        funnelerMotor.setVoltage(IntakeConstants.FUNNELER_VOLTAGE)
        conveyorMotor.setVoltage(IntakeConstants.CONVEYOR_INTAKE_VOLTAGE)
      }
    )
  }

  fun shoot(lowGoal: Boolean): Command {
    return Commands.sequence(
      runOnce {
        if (lowGoal) {
          shooterMotor.setControl(VelocityVoltage(IntakeConstants.SHOOTER_LOW_VELOCITY))
        } else {
          shooterMotor.setControl(VelocityVoltage(IntakeConstants.SHOOTER_HIGH_VELOCITY))
        }
      },
      WaitCommand(IntakeConstants.SHOOTER_SPINUP_TIME),
      runOnce {
        conveyorMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
        indexerMotor.setVoltage(IntakeConstants.INDEXER_VOLTAGE)
      }
    )
  }

  fun reject(): Command {
    return runOnce {
        intakeMotor.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
        funnelerMotor.setVoltage(-IntakeConstants.FUNNELER_VOLTAGE)
        conveyorMotor.setVoltage(-IntakeConstants.CONVEYOR_INTAKE_VOLTAGE)
    }
  }

  fun piecesShot(): Boolean {
    return detectsPiece(shooterSensor)
  }

  fun stop(): Command {
    return runOnce {
      conveyorMotor.stopMotor()
      intakeMotor.stopMotor()
      funnelerMotor.stopMotor()
      indexerMotor.stopMotor()
      shooterMotor.stopMotor()
    }
  }

  override fun periodic() {
//    if (!allSensorsConfigured && timer.hasElapsed(IntakeConstants.RECONFIGURE_WAIT_TIME)) {
//      //will shortcircuit if all configured so don't worry about expensive calc with has elapsed
//      //retry configuring sensors
//      configureSensors()
//    }
  }

  companion object {
    fun createIntake(): Intake {

      val intakeMotor = createKraken(
        IntakeConstants.INTAKE_MOTOR_ID,
        IntakeConstants.INTAKE_MOTOR_INVERTED
      )
      val funnelerLeaderMotor = createSparkMax(
        IntakeConstants.FUNNELER_LEADER_ID,
        IntakeConstants.FUNNELER_LEADER_INVERTED,
      )
      val funnelerFollowerMotor = createFollowerSpark(
        IntakeConstants.FUNNELER_FOLOWER_ID,
        funnelerLeaderMotor,
        IntakeConstants.FUNNELER_FOLLOWER_INVERSION,
      )

      val indexerLeaderMotor = createSparkMax(
        IntakeConstants.INDEXER_LEADER_ID,
        IntakeConstants.INDEXER_LEADER_INVERTED,
      )
      val indexerFollowerMotor = createFollowerSpark(
        IntakeConstants.INDEXER_FOLLOWER_ID,
        indexerLeaderMotor,
        IntakeConstants.INDEXER_FOLLOWER_INVERSION,
      )

      val conveyorMotor = createSparkMax(
        IntakeConstants.CONVEYOR_ID,
        IntakeConstants.CONVEYOR_INVERTED
      )

      val shooterMotor = createKraken(
        IntakeConstants.SHOOTER_ID,
        IntakeConstants.SHOOTER_INVERTED
      )

      val rightSensor = LaserCan(
        IntakeConstants.RIGHT_SENSOR_ID
      )
      val leftSensor = LaserCan(
        IntakeConstants.LEFT_SENSOR_ID
      )
      val shooterSensor = LaserCan(
        IntakeConstants.SHOOTER_SENSOR_ID
      )

      return Intake(
        intakeMotor,
        funnelerLeaderMotor,
        funnelerFollowerMotor,
        indexerLeaderMotor,
        indexerFollowerMotor,
        conveyorMotor,
        shooterMotor,
        rightSensor,
        leftSensor,
        shooterSensor
      )
    }
  }
}