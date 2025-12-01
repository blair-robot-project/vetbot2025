package frc.team449.subsystems.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax

class Intake(
  private val intakeLeader: TalonFX,
  follower1: TalonFX,
  private val firstIndexer: SparkMax,
  follower2: SparkMax,
  private val secondIndexer: SparkMax,
  follower3: SparkMax,
  private val conveyorMotor: SparkMax,
  private val shooterMotor: TalonFX,
  private val rightSensor: LaserCanInterface,
  private val leftSensor: LaserCanInterface,
  private val shooterSensor: LaserCanInterface
  ): SubsystemBase() {
  private val shootingDebouncer = Debouncer(IntakeConstants.SHOOTING_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling)

  private val sensors =
    listOf(
      leftSensor,
      rightSensor,
      shooterSensor,
    )

  private var allSensorsConfigured = true
  private var intakingSensorDown = false
  private var shootingSensorDown = false
  private var lasercanConfigured = listOf<Boolean>()

  init {
    try {
      for (sensor in sensors) {
        sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS)
        sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
        sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
        lasercanConfigured.plus(true)
      }
    } catch (_: Exception) {
      lasercanConfigured.plus(false)
      allSensorsConfigured = false
    }
    if(!lasercanConfigured[2]) {
      shootingSensorDown = true
    }
    if(!(lasercanConfigured[0] && lasercanConfigured[1])) { //DEMORGANSSSS
      intakingSensorDown = true
    }
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
        intakeLeader.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
        firstIndexer.setVoltage(IntakeConstants.FIRST_INDEXER_VOLTAGE)
        conveyorMotor.setVoltage(IntakeConstants.CONVEYOR_INTAKE_VOLTAGE)
      },
      ConditionalCommand (
        Commands.sequence(
          WaitUntilCommand { pieceIntaken() },
          WaitCommand(0.5),
        ),
        WaitCommand(IntakeConstants.NO_SENSOR_WAIT_TIME),
        { !intakingSensorDown }
      ),
      stop()
    )
  }

  fun shoot(lowGoal: Boolean): Command {
    return Commands.sequence(
      runOnce {
        if (lowGoal) {
          shooterMotor.setVoltage(IntakeConstants.SHOOTER_LOW_VOLTAGE)
        } else {
          shooterMotor.setVoltage(IntakeConstants.SHOOTER_HIGH_VOLTAGE)
        }
      },
      WaitCommand(IntakeConstants.SHOOTER_SPINUP_TIME),
      runOnce {
        conveyorMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
        secondIndexer.setVoltage(IntakeConstants.SECOND_INDEXER_VOLTAGE)
      },
      ConditionalCommand(
        Commands.sequence(
          WaitUntilCommand { piecesShot() },
          WaitUntilCommand { shootingDebouncer.calculate(!piecesShot())} // falling edge
        ),
        WaitCommand(IntakeConstants.NO_SENSOR_WAIT_TIME),
        { !shootingSensorDown }
      ),
      stop()
    )
  }

  fun reject(): Command {
    return Commands.sequence(
      runOnce {
        intakeLeader.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
        firstIndexer.setVoltage(-IntakeConstants.FIRST_INDEXER_VOLTAGE)
        conveyorMotor.setVoltage(-IntakeConstants.CONVEYOR_INTAKE_VOLTAGE)
      },
      WaitCommand(2.0),
      stop()
    )
  }

  fun piecesShot(): Boolean {
    return detectsPiece(shooterSensor)
  }

  fun stop(): Command {
    return runOnce {
      conveyorMotor.stopMotor()
      intakeLeader.stopMotor()
      firstIndexer.stopMotor()
      secondIndexer.stopMotor()
      shooterMotor.stopMotor()
    }
  }

  companion object {
    fun createIntake(): Intake {

      val intakeLeader = createKraken(
        IntakeConstants.INTAKE_LEADER_ID,
        IntakeConstants.INTAKE_LEADER_INVERTED
      )

      val intakeFollower = TalonFX(
        IntakeConstants.INTAKE_FOLLOWER_ID
      )
      intakeFollower.setControl(
        Follower(IntakeConstants.INTAKE_LEADER_ID, IntakeConstants.INTAKE_FOLLOWER_INVERSION)
      )

      val firstIndexerLeaderMotor = createSparkMax(
        IntakeConstants.FIRST_INDEXER_LEADER_ID,
        IntakeConstants.FIRST_INDEXER_LEADER_INVERTED,
      )
      val firstIndexerFollowerMotor = createFollowerSpark(
        IntakeConstants.FIRST_INDEXER_FOLLOWER_ID,
        firstIndexerLeaderMotor,
        IntakeConstants.FIRST_INDEXER_FOLLOWER_INVERSION,
      )

      val secondIndexerLeaderMotor = createSparkMax(
        IntakeConstants.SECOND_INDEXER_LEADER_ID,
        IntakeConstants.SECOND_INDEXER_LEADER_INVERTED,
      )
      val secondIndexerFollowerMotor = createFollowerSpark(
        IntakeConstants.SECOND_INDEXER_FOLLOWER_ID,
        secondIndexerLeaderMotor,
        IntakeConstants.SECOND_INDEXER_FOLLOWER_INVERSION,
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
        intakeLeader,
        intakeFollower,
        firstIndexerLeaderMotor,
        firstIndexerFollowerMotor,
        secondIndexerLeaderMotor,
        secondIndexerFollowerMotor,
        conveyorMotor,
        shooterMotor,
        rightSensor,
        leftSensor,
        shooterSensor
      )
    }
  }
}