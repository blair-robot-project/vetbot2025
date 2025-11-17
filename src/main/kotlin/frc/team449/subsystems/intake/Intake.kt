package frc.team449.subsystems.intake

import au.grapplerobotics.LaserCan
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax

class Intake(
  val wheelLeader: SparkMax,
  val wheelFollower: SparkMax,
  val pieceSensor: LaserCan,
  val rollerMotor: TalonFX
  ): SubsystemBase() {

  fun intake(): Command {
    return runOnce {
      rollerMotor.setVoltage(IntakeConstants.ROLLER_INTAKE_VOLTAGE)
      wheelLeader.setVoltage(IntakeConstants.WHEEL_INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return runOnce {
      rollerMotor.setVoltage(IntakeConstants.ROLLER_OUTTAKE_VOLTAGE)
      wheelLeader.setVoltage(IntakeConstants.WHEEL_OUTTAKE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return runOnce {
      rollerMotor.stopMotor()
      wheelLeader.stopMotor()
    }
  }

  companion object {
    fun createIntake(): Intake {
      val wheelLeader = createSparkMax(
        IntakeConstants.WHEEL_LEADER_ID,
        IntakeConstants.WHEEL_LEADER_INVERTED,
      )
      val wheelFollower = createFollowerSpark(
        IntakeConstants.WHEEL_FOLLOWER_ID,
        wheelLeader,
        IntakeConstants.WHEEL_FOLLOWER_INVERSION
      )
      val rollerMotor = createKraken(
        IntakeConstants.ROLLER_ID,
        IntakeConstants.ROLLER_INVERTED,
      )
      val pieceSensor = LaserCan(
        IntakeConstants.SENSOR_ID
      )
      return Intake(
        wheelLeader,
        wheelFollower,
        pieceSensor,
        rollerMotor
      )
    }
  }
}