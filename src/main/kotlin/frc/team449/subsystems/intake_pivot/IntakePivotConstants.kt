package frc.team449.subsystems.intake_pivot

import edu.wpi.first.units.Units.*

object IntakePivotConstants {
  const val PIVOT_ID = 3 // TODO: find
  const val PIVOT_INVERTED = false // TODO: find
  const val PIVOT_KP = 1.0
  const val PIVOT_KI = 0.0
  const val PIVOT_KD = 0.0
  const val PIVOT_KS = 0.0
  const val PIVOT_KV = 0.0
  const val PIVOT_KA = 0.0
  const val PIVOT_KG = 0.0
  const val PIVOT_SENSOR_TO_MECH = 11.0 / 3.0
  val PIVOT_CRUISE_VEL = RotationsPerSecond.of(1.0)
  val PIVOT_MAX_ACCEL = RotationsPerSecondPerSecond.of(1.0)
}