package frc.team449.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.intake_pivot.IntakePivotConstants
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax

class Intake(
  val intake_wheel_motor_leader: SparkMax,
  val intake_wheel_motor_follower: SparkMax,
  val roller_motor: TalonFX
  ): SubsystemBase() {

  fun intake(): Command {
    return runOnce {
      roller_motor.setVoltage(IntakeConstants.ROLLER_INTAKE_VOLTAGE)
      intake_wheel_motor_leader.setVoltage(IntakeConstants.INTAKE_WHEEL_INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return runOnce {
      roller_motor.setVoltage(IntakeConstants.ROLLER_OUTTAKE_VOLTAGE)
      intake_wheel_motor_leader.setVoltage(IntakeConstants.INTAKE_WHEEL_OUTTAKE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return runOnce {
      roller_motor.stopMotor()
      intake_wheel_motor_leader.stopMotor()
    }
  }

  companion object {
    fun createIntake(): Intake {
      val intake_wheel_motor_leader = createSparkMax(
        IntakeConstants.INTAKE_WHEEL_LEADER_ID,
        IntakeConstants.INTAKE_WHEEL_LEADER_INVERTED,
      )
      val intake_wheel_motor_follower = createFollowerSpark(
        IntakeConstants.INTAKE_WHEEL_FOLLOWER_ID,
        intake_wheel_motor_leader,
        IntakeConstants.INTAKE_WHEEL_FOLLOWER_INVERTED_FROM_LEADER
      )
      val roller_motor = createKraken(
        IntakeConstants.ROLLER_ID,
        IntakeConstants.ROLLER_INVERTED,
      )
      return Intake(
        intake_wheel_motor_leader,
        intake_wheel_motor_follower,
        roller_motor
      )
    }
  }
}