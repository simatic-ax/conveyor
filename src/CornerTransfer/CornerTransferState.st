NAMESPACE Simatic.Ax.Conveyor
    TYPE
        /// States of the CornerTransfer sequence
        CornerTransferState : (
            Idle,
            Receiving,
            LiftingUp,
            ReadyToDeliver,
            Delivering,
            LoweringDown
        ) := Idle;
    END_TYPE
END_NAMESPACE
