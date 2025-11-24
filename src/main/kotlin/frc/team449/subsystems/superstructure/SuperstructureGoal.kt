package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.*
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive

object SuperstructureGoal {

  val STOW = SuperstructureState(
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Stow"
  )

  val INTAKE = SuperstructureState(
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Intake"
  )

  data class SuperstructureState(
    val driveDynamics: DriveDynamics,
    val name: String
  )

  data class DriveDynamics(
    val maxSpeed: Double,
    val maxAccel: Double,
    val maxRotSpeed: Double
  )

  fun applyDriveDynamics(drive: SwerveDrive, dynamics: DriveDynamics) {
    drive.maxLinearSpeed = dynamics.maxSpeed
    drive.accel = dynamics.maxAccel
    drive.maxRotSpeed = dynamics.maxRotSpeed
  }
}
