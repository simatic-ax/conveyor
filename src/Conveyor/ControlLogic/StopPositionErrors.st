NAMESPACE Simatic.Ax.Conveyor

    /// Error codes specific to StopPosition supervision.
    TYPE
        StopPositionError : WORD (
            NoError := WORD#16#0000,
            SensorBlocked := WORD#16#8120
        ) := NoError;
    END_TYPE

END_NAMESPACE