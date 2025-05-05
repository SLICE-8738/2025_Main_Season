## AlignAndGetAlgae
    This command aligns the robot with the reef and gets a piece of algae.

    It exists to help get algae from the reef.

## AlignAndGetCoral
    This command aligns the robot with the coral station and gets a piece of coral.

    It exists to help get coral from the coral station.

## AlignAndGetCoralAutonomous
    This command aligns the robot with the coral station and gets a piece of coral during autonomous.

    It exists to help get coral from the coral station during autonomous.

## AlignAndScoreCoral
    This command aligns the robot with the reef and places a piece of coral on a level.

    It exists to help place coral in the reef.

## AlignAndScoreCoralAutonomous
    This command aligns the robot with the reef and places a piece of coral on a level during autonomous.

    It exists to help place coral in the reef during autonomous.

## BargeAlgae
    This command raises the elevator to the top level to align with the barge.

    It exists to help place algae in the barge.

## IntakeAdjustment
    This command helps to adjust the intake if coral is stuck in the end effector.

    It exists to help stop coral getting stuck in the end effector

## MoveToLevel
    This command helps to move the robot to a level. It has arguments to retain algae in the end effector, and to align the end effector first. This is done sequentially in order to prevent hitting the top of the reef.

    It exists to help move the elevator and end effector to the correct position.

## MoveToLevelParallel
    This command helps to move the elevator to a level, and move the end effector simultaneously. It does not have options to retain algae.

    It exists to help move the elevator and end effector the the correct position quickly.

## PickupAlgaePhase1
    This command helps to intake algae using MoveToLevelParallel.

    It exists to help intake algae.

## PickupAlgaePhase2
    This command helps to intake algae, primarily during autonomous.

    It exists to help intake algae.