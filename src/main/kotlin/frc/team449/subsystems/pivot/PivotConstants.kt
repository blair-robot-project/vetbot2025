package frc.team449.subsystems.pivot

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency

object PivotConstants {


  const val PIVOT_ID = 3 // TODO: find
  const val PIVOT_INVERTED = false // TODO: find

  const val KP = 5.5
  const val KI = 0.0
  const val KD = 0.5

  // TODO(Adjust gains.)
  const val KS = 0.085813
  const val KV = 4.4941
  const val KG = 0.15439

  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)
  val INTAKE_WEIGHT_KG = 5.0 // TODO: find

  val PIVOT_CRUISE_VEL = RotationsPerSecond.of(0.2767) // max theoretical 0.3968  // 0.365 norm
  val PIVOT_MAX_ACCEL = RotationsPerSecondPerSecond.of(1.8) // 5.0, heavily limited by robot tipping // 2.125
  val TOLERANCE = Degrees.of(2.0)
}