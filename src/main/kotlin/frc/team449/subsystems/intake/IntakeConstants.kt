package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.RotationsPerSecond

object IntakeConstants {
  const val INTAKE_MOTOR_ID = 11
  const val INTAKE_MOTOR_INVERTED = false

  const val FUNNELER_LEADER_ID = 12
  const val FUNNELER_LEADER_INVERTED = false
  const val FUNNELER_FOLOWER_ID = 13
  const val FUNNELER_FOLLOWER_INVERSION = true

  const val INDEXER_LEADER_ID = 14
  const val INDEXER_LEADER_INVERTED = false
  const val INDEXER_FOLLOWER_ID = 15
  const val INDEXER_FOLLOWER_INVERSION = true

  const val CONVEYOR_ID = 9
  const val CONVEYOR_INVERTED = false

  const val SHOOTER_ID = 16
  const val SHOOTER_INVERTED = false

  const val LEFT_SENSOR_ID = 20
  const val RIGHT_SENSOR_ID = 21
  const val SHOOTER_SENSOR_ID = 22

  const val INTAKE_VOLTAGE = 4.0

  const val CONVEYOR_INTAKE_VOLTAGE = 4.0
  const val FUNNELER_VOLTAGE = 5.0
  const val INDEXER_VOLTAGE = 5.0

  const val CORAL_DETECTION_THRESHOLD = 50

  val SHOOTER_HIGH_VELOCITY = RotationsPerSecond.of(4000.0 / 60.0)
  val SHOOTER_LOW_VELOCITY = RotationsPerSecond.of(2000.0 / 60.0)

  const val SHOOTER_SPINUP_TIME = 0.5 //seconds
  const val SHOOTING_DEBOUNCE_TIME = 1.0 //seconds
  const val NO_SENSOR_WAIT_TIME = 2.0 //seconds
  const val RECONFIGURE_WAIT_TIME = 10.0 //seconds
}