package frc.team449.subsystems.shooter
import edu.wpi.first.units.Units.*

object ShooterConstants {
  const val INTAKE_LEADER_ID = 6
  const val INTAKE_LEADER_INVERTED = false
  const val INTAKE_FOLLOWER_ID = 7
  const val INTAKE_FOLLOWER_INVERTED = true

  const val SHOOTER_ID = 8
  const val SHOOTER_INVERTED = false
  const val SHOOTER_KS = 0.0
  const val SHOOTER_KV = 0.0
  const val SHOOTER_KA = 0.0
  const val SHOOTER_KP = 1.0
  const val SHOOTER_KI = 0.0
  const val SHOOTER_KD = 0.0

  const val LASER_CAN_ID = 9

  const val INTAKE_VOLTAGE = 4.0
  const val OUTTAKE_VOLTAGE = 6.0
  val SHOOTER_VELOCITY_TARGET = RotationsPerSecond.of(4000.0 / 60.0)
  const val DETECTION_TARGET_MM = 50.0

}