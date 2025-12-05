package frc.team449.subsystems.pivot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.createKraken
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.cos

@Logged
class Pivot (
  val motor: TalonFX,
) : SubsystemBase() {

  private val timer = Timer()
  val positionSupplier = Supplier { motor.position.valueAsDouble }

  private val request: MotionMagicVoltage = MotionMagicVoltage(
    SuperstructureGoal.STOW.pivot.`in`(Radians)
  ).withEnableFOC(false)

  fun resetPos(): Command {
    return runOnce {
      motor.setPosition(PivotConstants.TRUE_STOW_ANGLE)
    }
  }

  fun setPosition(position: Double): Command {
    return runOnce {
      if(position < positionSupplier.get()) {
        val config = TalonFXConfiguration()
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast
        motor.configurator.apply(config)
      }
    }.andThen(
      runOnce {
        motor.setControl(
          request
            .withPosition(position)
            .withUpdateFreqHz(PivotConstants.REQUEST_UPDATE_RATE)
        )
      }
    )
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setControl(
        PositionVoltage(positionSupplier.get())
          .withUpdateFreqHz(PivotConstants.REQUEST_UPDATE_RATE)
      )
    }
  }

  fun currentHoming(): Command {
    return FunctionalCommand(
      {
        timer.stop()
        timer.reset()
      },
      {
        motor.setVoltage(PivotConstants.HOMING_VOLTAGE.magnitude())
        if (motor.statorCurrent.valueAsDouble > PivotConstants.HOMING_CURRENT_CUTOFF.`in`(Amps) &&
          motor.velocity.valueAsDouble < PivotConstants.HOMING_MAX_VEL.`in`(RotationsPerSecond)
        ) {
          timer.start()
        } else {
          timer.stop()
          timer.reset()
        }
      },
      {
        motor.setPosition(PivotConstants.TRUE_STOW_ANGLE.`in`(Rotations))
        println("COMPLETED PIVOT CURRENT HOMING, SET TO STOW ANGLE")
      },
      {
        motor.statorCurrent.valueAsDouble > PivotConstants.HOMING_CURRENT_CUTOFF.`in`(Amps) &&
          timer.hasElapsed(PivotConstants.HOMING_TIME_CUTOFF.`in`(Seconds))
      }
    )
  }

  fun atSetpoint(): Boolean {
    return (abs(motor.position.valueAsDouble - request.Position) < PivotConstants.TOLERANCE.`in`(Radians))
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  companion object {
    fun createPivot() : Pivot {
      val pivotMotor = createKraken(
        PivotConstants.PIVOT_ID,
        PivotConstants.PIVOT_INVERTED,
        kS = PivotConstants.KS,
        kV = PivotConstants.KV,
        kG = PivotConstants.KG,
        kP = PivotConstants.KP,
        kI = PivotConstants.KI,
        kD = PivotConstants.KD,
        gravityType = GravityTypeValue.Arm_Cosine,
        cruiseVel = PivotConstants.PIVOT_CRUISE_VEL.`in`(RotationsPerSecond),
        maxAccel = PivotConstants.PIVOT_MAX_ACCEL.`in`(RotationsPerSecondPerSecond)
      )
      return Pivot(pivotMotor)
    }
  }
}
