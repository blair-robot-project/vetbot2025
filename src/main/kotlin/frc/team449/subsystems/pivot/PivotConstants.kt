package frc.team449.subsystems.pivot

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency

object PivotConstants {

  const val PIVOT_ID = 10
  const val PIVOT_INVERTED = false // TODO: find

  const val KP = 15.5
  const val KI = 0.0
  const val KD = 0.0

  // TODO(Adjust gains.)
  const val KS = 0.085813
  const val KV = 6.4941
  const val KG = 0.15439

  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)
  val INTAKE_WEIGHT_KG = 5.0 // TODO: find

  /** Current Homing constants */
  val HOMING_VOLTAGE = Volts.of(-2.0)
  val HOMING_TIME_CUTOFF = Seconds.of(4.0)
  val HOMING_CURRENT_CUTOFF = Amps.of(20.0)
  val HOMING_MAX_VEL = RotationsPerSecond.of(0.05)
  val TRUE_STOW_ANGLE = Rotations.of(0.23583984375)

  val PIVOT_CRUISE_VEL = RotationsPerSecond.of(0.5) // max theoretical 0.3968  // 0.365 norm
  val PIVOT_MAX_ACCEL = RotationsPerSecondPerSecond.of(1.8) // 5.0, heavily limited by robot tipping // 2.125
  val TOLERANCE = Degrees.of(5.0)

  const val PIVOT_SENSOR_TO_MECH = 891.0 / 40.0
}