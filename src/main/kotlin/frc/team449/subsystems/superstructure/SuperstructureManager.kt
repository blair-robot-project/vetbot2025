package frc.team449.subsystems.superstructure

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.intake.Intake
import frc.team449.subsystems.pivot.Pivot

/*
* pivot goes down
* pivot goes to stow
* you run intake
* run the conveyor a little to make sure it goes in
* count the number of pieces in superstructure
* RUN FLYWHEELS WHOLE TIME (its good)
* don't run index whole time
* when they click shoot:
* run conveyor and run
* ONE ON CONVEYOR
* TWO FOR INDEXOR (one running opposite way )
* ONE FOR SHOOTER (its like connected or smth)
* TWO FOR INTAKE (one running oppposite)
* TWO FOR PIVOT
* */

@Logged
class SuperstructureManager(
  @NotLogged
  private val drive: SwerveDrive,
  @NotLogged
  private val driveCommand: SwerveOrthogonalCommand,
  @NotLogged
  private val intake: Intake,
  @NotLogged
  private val pivot: Pivot
) {

  private var command = "stow"
  @Logged(name = "current command")
  fun logCommand(): String {
    return command
  }

  fun prepIntake(): Command {
    return Commands.sequence(
      InstantCommand ({
        SuperstructureGoal.applyDriveDynamics(drive, SuperstructureGoal.INTAKE.driveDynamics)
        command = "moving to intake"
      }),
      pivot.setPosition(SuperstructureGoal.INTAKE.pivot.`in`(Radians)),
      WaitUntilCommand { pivot.atSetpoint() },
      pivot.hold(),
      InstantCommand ({ command = "nothing" })
    )
  }

  fun intake(): Command {
    return Commands.sequence(
      InstantCommand ({ command = "intaking" }),
      intake.intake(),
      InstantCommand ({ command = "nothing" }),
      stow().onlyIf { intake.pieces == 3 }
    )
  }

  fun stow(): Command {
    return Commands.sequence(
      intake.stop(),
        InstantCommand ({
          SuperstructureGoal.applyDriveDynamics(drive, SuperstructureGoal.STOW.driveDynamics)
          command = "stowing"
        }),
      pivot.setPosition(SuperstructureGoal.INTAKE.pivot.`in`(Radians)),
      WaitUntilCommand { pivot.atSetpoint() },
      pivot.hold(),
      InstantCommand ({ command = "nothing" })
    )
  }

  fun shootLow(): Command {
    return Commands.sequence(
      InstantCommand({ command = "shooting low "}),
      intake.shoot(true),
      InstantCommand ({ command = "nothing" })
    )
  }

  fun shootHigh(): Command {
    return Commands.sequence(
      InstantCommand({ command = "shooting high "}),
      intake.shoot(false),
      InstantCommand ({ command = "nothing" })
    )
  }

  fun rejectPiece(): Command {
    return intake.reject()
  }

  companion object {
    fun createSuperstructureManager(robot: Robot): SuperstructureManager {
      return SuperstructureManager(
        robot.drive,
        robot.driveCommand,
        robot.intake,
        robot.pivot
      )
    }
  }
}
