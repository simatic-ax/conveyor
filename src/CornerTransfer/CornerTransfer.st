USING Simatic.Ax.IO.Input;
USING Simatic.Ax.Motor;
USING Simatic.Ax.AutomationBase;
USING System.Timer;

NAMESPACE Simatic.Ax.Conveyor

    /// Corner transfer unit: receives goods on rollers, lifts them onto a
    /// transverse conveyor and delivers them to the next conveyor (90° turn).
    ///
    ///           Prev
    ///            ||
    ///   ________ \/ ________
    ///  |  StopSensor        |
    ///  |   Rollers   ->     | ==> Transverse conveyor (Motor) ==> Next
    ///  |  RollerEndSensor   |
    ///  |____________________|
    ///         Lift (LiftMotor / LiftUpSensor)
    ///
    /// Sequence: Idle -> Receiving -> LiftingUp -> ReadyToDeliver -> Delivering -> LoweringDown -> Idle
    ///
    /// Interface and usage follow ConveyorBase (IConveyor, EquipmentBase).
    CLASS CornerTransfer
        EXTENDS EquipmentBase
        IMPLEMENTS IConveyor
        VAR CONSTANT
            /// Default normal operating speed in m/s
            DEFAULT_AUTOMATIC_SPEED : LREAL := 1.5;
            /// Default slow speed for precise positioning in m/s
            DEFAULT_MANUAL_SPEED : LREAL := 0.5;
            /// Default motor acceleration in m/s²
            DEFAULT_ACCELERATION : LREAL := 1.0;
            /// Default motor deceleration in m/s²
            DEFAULT_DECELERATION : LREAL := 1.0;
        END_VAR
        VAR PUBLIC
            /// Corner transfer identifier name
            Name : STRING[30];
            /// Entry sensor, same role as ConveyorBase.StopSensor (required)
            StopSensor : IBinSignal;
            /// TRUE when the goods reached the end of the rollers (required)
            RollerEndSensor : IBinSignal;
            /// TRUE when the lift is in upper position (required)
            LiftUpSensor : IBinSignal;
            /// Transverse conveyor towards the exit (uses NullMotor if NULL)
            Motor : IMotor;
            /// Roller conveyor for receiving (uses NullMotor if NULL)
            RollerMotor : IMotor;
            /// Lift drive; lift is lowered when this motor is off and LiftUpSensor is FALSE (uses NullMotor if NULL)
            LiftMotor : IMotor;
            /// Reference to previous (upstream) conveyor
            PrevConveyor : IDeliverer;
            /// Reference to next (downstream) conveyor (uses NullReceiver if NULL)
            NextConveyor : IReceiver;
            /// Transverse conveyor follow-up time after goods left the sensors
            OffDelayTime : TIME := T#5s;
            /// Normal operating speed in m/s
            AutomaticSpeed : LREAL := DEFAULT_AUTOMATIC_SPEED;
            /// Slow speed in manual mode in m/s
            ManualSpeed : LREAL := DEFAULT_MANUAL_SPEED;
            /// Motor acceleration rate in m/s²
            Acceleration : LREAL := DEFAULT_ACCELERATION;
            /// Motor deceleration rate in m/s²
            Deceleration : LREAL := DEFAULT_DECELERATION;
            /// Supervision time for each sequence step
            SensorBlockedTime : LTIME := T#5s;
        END_VAR

        VAR PRIVATE
            _state : CornerTransferState := CornerTransferState#Idle;
            _configOk : BOOL;
            _nullMotor : NullMotor;
            _nullRollerMotor : NullMotor;
            _nullLiftMotor : NullMotor;
            _nullReceiver : NullReceiver;
            _motorRunning : BOOL;
            _rollerRunning : BOOL;
            _liftRunning : BOOL;
            _manualStart : BOOL;
            _triggerStart : BOOL;
            _stepTimer : OnDelay;
            _offDelayTimer : OnDelay;
            _triggerTimer : OnDelay;
        END_VAR

        METHOD PROTECTED OVERRIDE InitUser
            IF Motor = NULL THEN
                Motor := _nullMotor;
            END_IF;
            IF RollerMotor = NULL THEN
                RollerMotor := _nullRollerMotor;
            END_IF;
            IF LiftMotor = NULL THEN
                LiftMotor := _nullLiftMotor;
            END_IF;
            IF NextConveyor = NULL THEN
                NextConveyor := _nullReceiver;
            END_IF;

            _configOk := TRUE;
            IF StopSensor = NULL OR RollerEndSensor = NULL OR LiftUpSensor = NULL THEN
                _configOk := FALSE;
                THIS.SetError(errorState := ErrorState#ConfigError, errorText := 'Sensor not configured');
            END_IF;
            IF ExternalRelease = NULL THEN
                _configOk := FALSE;
                THIS.SetError(errorState := ErrorState#ConfigError, errorText := 'Release not configured');
            END_IF;
        END_METHOD

        METHOD PROTECTED OVERRIDE RunCyclicUserCode
            VAR_TEMP
                stepEnabled : BOOL;
                isAutomatic : BOOL;
                itemPresent : BOOL;
                runRoller : BOOL;
                runLift : BOOL;
                runMotor : BOOL;
                supervise : BOOL;
                errorText : STRING[30];
            END_VAR

            IF NOT _configOk OR THIS.HasError() THEN
                THIS.StopAllMotors();
                THIS.UpdateMotors();
                RETURN;
            END_IF;

            isAutomatic := Mode.GetOperatingMode() = OperatingModes#Automatic;
            IF Mode.GetOperatingMode() <> OperatingModes#Manual THEN
                _manualStart := FALSE;
            END_IF;
            IF isAutomatic THEN
                stepEnabled := THIS.GetStartupStatus() = StartUpStatus#AutoStartedUp;
            ELSE
                stepEnabled := _manualStart AND ExternalRelease.IsReleased();
            END_IF;

            itemPresent := StopSensor.Q() OR RollerEndSensor.Q();

            IF stepEnabled THEN
                CASE _state OF
                    CornerTransferState#Idle:
                        runRoller := THIS.LiftIsDown() AND (_triggerStart OR (isAutomatic AND THIS.PrevIsDelivering()));
                        IF RollerEndSensor.Q() THEN
                            THIS.SetState(CornerTransferState#LiftingUp);
                        ELSIF StopSensor.Q() THEN
                            THIS.SetState(CornerTransferState#Receiving);
                        END_IF;

                    CornerTransferState#Receiving:
                        runRoller := TRUE;
                        supervise := TRUE;
                        errorText := 'Roller end timeout';
                        IF RollerEndSensor.Q() THEN
                            THIS.SetState(CornerTransferState#LiftingUp);
                        END_IF;

                    CornerTransferState#LiftingUp:
                        runLift := TRUE;
                        supervise := TRUE;
                        errorText := 'Lift up timeout';
                        IF LiftUpSensor.Q() THEN
                            THIS.SetState(CornerTransferState#ReadyToDeliver);
                        END_IF;

                    CornerTransferState#ReadyToDeliver:
                        runLift := TRUE;
                        IF NOT itemPresent THEN
                            // Goods removed manually
                            THIS.SetState(CornerTransferState#LoweringDown);
                        ELSIF NOT LiftUpSensor.Q() THEN
                            THIS.SetState(CornerTransferState#LiftingUp);
                        ELSIF NOT isAutomatic OR THIS.NextIsReadyToReceive() THEN
                            THIS.SetState(CornerTransferState#Delivering);
                        END_IF;

                    CornerTransferState#Delivering:
                        runLift := TRUE;
                        runMotor := TRUE;
                        supervise := itemPresent;
                        errorText := 'Delivery timeout';
                        IF itemPresent AND NOT LiftUpSensor.Q() THEN
                            THIS.SetState(CornerTransferState#LiftingUp);
                        END_IF;

                    CornerTransferState#LoweringDown:
                        supervise := TRUE;
                        errorText := 'Lift down timeout';
                        IF NOT LiftUpSensor.Q() THEN
                            THIS.SetState(CornerTransferState#Idle);
                        END_IF;
                END_CASE;
            END_IF;

            _stepTimer(signal := stepEnabled AND supervise, duration := SensorBlockedTime);
            IF _stepTimer.output THEN
                THIS.SetError(errorState := ErrorState#OtherError, errorText := errorText);
                THIS.StopAllMotors();
                THIS.UpdateMotors();
                RETURN;
            END_IF;

            // Transverse conveyor keeps running for OffDelayTime after goods left the sensors
            _offDelayTimer(signal := stepEnabled AND _state = CornerTransferState#Delivering AND NOT itemPresent,
                           duration := OffDelayTime);
            IF _offDelayTimer.output THEN
                runMotor := FALSE;
                THIS.SetState(CornerTransferState#LoweringDown);
            END_IF;

            _triggerTimer(signal := _triggerStart, duration := OffDelayTime);
            IF _triggerTimer.output OR _state <> CornerTransferState#Idle THEN
                _triggerStart := FALSE;
            END_IF;

            THIS.DriveMotor(drive := RollerMotor, run := runRoller, wasRunning := _rollerRunning);
            THIS.DriveMotor(drive := LiftMotor, run := runLift, wasRunning := _liftRunning);
            THIS.DriveMotor(drive := Motor, run := runMotor, wasRunning := _motorRunning);
            THIS.UpdateMotors();
        END_METHOD

        /// Resets the fault state and resumes the sequence from the physical sensor state
        METHOD PROTECTED OVERRIDE ResetFaultUser : BOOL
            THIS.StopAllMotors();
            _manualStart := FALSE;
            _triggerStart := FALSE;
            _offDelayTimer(signal := FALSE, duration := OffDelayTime);
            _triggerTimer(signal := FALSE, duration := OffDelayTime);

            IF NOT _configOk THEN
                THIS.SetState(CornerTransferState#Idle);
            ELSIF LiftUpSensor.Q() AND (StopSensor.Q() OR RollerEndSensor.Q()) THEN
                THIS.SetState(CornerTransferState#ReadyToDeliver);
            ELSIF RollerEndSensor.Q() THEN
                THIS.SetState(CornerTransferState#LiftingUp);
            ELSIF StopSensor.Q() THEN
                THIS.SetState(CornerTransferState#Receiving);
            ELSIF LiftUpSensor.Q() THEN
                THIS.SetState(CornerTransferState#LoweringDown);
            ELSE
                THIS.SetState(CornerTransferState#Idle);
            END_IF;
            ResetFaultUser := TRUE;
        END_METHOD

        /// Manually triggers the roller conveyor in automatic mode
        /// Only works if AutoStartedUp, Idle and not occupied
        METHOD PUBLIC TriggerStartInAutomatic
            IF THIS.GetStartupStatus() = StartUpStatus#AutoStartedUp
                AND _state = CornerTransferState#Idle
                AND NOT THIS.IsOccupied() THEN
                _triggerStart := TRUE;
            END_IF;
        END_METHOD

        /// Starts the sequence in manual mode
        /// Requires manual operating mode and external release
        METHOD PUBLIC StartManual : BOOL
            IF Mode <> NULL AND ExternalRelease <> NULL THEN
                IF Mode.GetOperatingMode() = OperatingModes#Manual AND ExternalRelease.IsReleased() THEN
                    _manualStart := TRUE;
                END_IF;
            END_IF;
            StartManual := _manualStart;
        END_METHOD

        /// Stops the sequence; all motors stop in the next cycle
        METHOD PUBLIC Stop
            _manualStart := FALSE;
            _triggerStart := FALSE;
        END_METHOD

        METHOD PUBLIC DeliverStatus : DeliverState
            DeliverStatus := DeliverState#Idle;
            IF NOT THIS.HasError() THEN
                CASE _state OF
                    CornerTransferState#ReadyToDeliver:
                        DeliverStatus := DeliverState#ReadyToDeliver;
                    CornerTransferState#Delivering:
                        DeliverStatus := DeliverState#Delivering;
                END_CASE;
            END_IF;
        END_METHOD

        METHOD PUBLIC ReceiveStatus : ReceiveState
            ReceiveStatus := ReceiveState#Idle;
            IF NOT _configOk OR THIS.HasError() THEN
                RETURN;
            END_IF;
            IF _state = CornerTransferState#Receiving THEN
                ReceiveStatus := ReceiveState#Receiving;
            ELSIF THIS.GetStartupStatus() = StartUpStatus#AutoStartedUp
                AND _state = CornerTransferState#Idle
                AND THIS.LiftIsDown()
                AND NOT THIS.IsOccupied() THEN
                ReceiveStatus := ReceiveState#ReadyToReceive;
            END_IF;
        END_METHOD

        METHOD PUBLIC IsOccupied : BOOL
            IsOccupied := _state <> CornerTransferState#Idle AND _state <> CornerTransferState#LoweringDown;
            IF _configOk THEN
                IsOccupied := IsOccupied OR StopSensor.Q() OR RollerEndSensor.Q();
            END_IF;
        END_METHOD

        METHOD PUBLIC StopSensorStatus : BOOL
            IF _configOk THEN
                StopSensorStatus := StopSensor.Q();
            END_IF;
        END_METHOD

        METHOD PUBLIC Delivering : BOOL
            Delivering := _state = CornerTransferState#Delivering;
        END_METHOD

        /// Refers to the transverse conveyor motor, like ConveyorBase.ConveyorIsRunning
        METHOD PUBLIC ConveyorIsRunning : BOOL
            IF Motor <> NULL THEN
                ConveyorIsRunning := Motor.IsRunning();
            END_IF;
        END_METHOD

        METHOD PUBLIC GetMotorSpeed : LREAL
            IF Motor <> NULL THEN
                GetMotorSpeed := Motor.GetCurrentSpeed();
            END_IF;
        END_METHOD

        METHOD PUBLIC RollerIsRunning : BOOL
            IF RollerMotor <> NULL THEN
                RollerIsRunning := RollerMotor.IsRunning();
            END_IF;
        END_METHOD

        METHOD PUBLIC LiftIsRunning : BOOL
            IF LiftMotor <> NULL THEN
                LiftIsRunning := LiftMotor.IsRunning();
            END_IF;
        END_METHOD

        METHOD PUBLIC GetState : CornerTransferState
            GetState := _state;
        END_METHOD

        METHOD PRIVATE SetState
            VAR_INPUT
                newState : CornerTransferState;
            END_VAR
            IF newState <> _state THEN
                _stepTimer(signal := FALSE, duration := SensorBlockedTime);
                _state := newState;
            END_IF;
        END_METHOD

        METHOD PRIVATE LiftIsDown : BOOL
            LiftIsDown := NOT _liftRunning AND NOT LiftUpSensor.Q();
        END_METHOD

        METHOD PRIVATE PrevIsDelivering : BOOL
            VAR_TEMP
                prevStatus : DeliverState;
            END_VAR
            IF PrevConveyor <> NULL THEN
                prevStatus := PrevConveyor.DeliverStatus();
                PrevIsDelivering := prevStatus = DeliverState#ReadyToDeliver OR prevStatus = DeliverState#Delivering;
            END_IF;
        END_METHOD

        METHOD PRIVATE NextIsReadyToReceive : BOOL
            NextIsReadyToReceive := NextConveyor.ReceiveStatus() = ReceiveState#ReadyToReceive;
        END_METHOD

        /// Starts/stops a motor on edges of the run request
        METHOD PRIVATE DriveMotor
            VAR_INPUT
                drive : IMotor;
                run : BOOL;
            END_VAR
            VAR_IN_OUT
                wasRunning : BOOL;
            END_VAR
            IF run AND NOT wasRunning THEN
                IF Mode.GetOperatingMode() = OperatingModes#Manual THEN
                    drive.StartMotor(targetSpeed := ManualSpeed, acceleration := Acceleration);
                ELSE
                    drive.StartMotor(targetSpeed := AutomaticSpeed, acceleration := Acceleration);
                END_IF;
            ELSIF NOT run AND wasRunning THEN
                drive.StopMotor(deceleration := Deceleration);
            END_IF;
            wasRunning := run;
        END_METHOD

        METHOD PRIVATE StopAllMotors
            IF Motor <> NULL AND RollerMotor <> NULL AND LiftMotor <> NULL THEN
                THIS.DriveMotor(drive := Motor, run := FALSE, wasRunning := _motorRunning);
                THIS.DriveMotor(drive := RollerMotor, run := FALSE, wasRunning := _rollerRunning);
                THIS.DriveMotor(drive := LiftMotor, run := FALSE, wasRunning := _liftRunning);
            END_IF;
        END_METHOD

        /// Cyclic ramp update of all motors, see IMotor.Update
        METHOD PRIVATE UpdateMotors
            IF Motor <> NULL AND RollerMotor <> NULL AND LiftMotor <> NULL THEN
                Motor.Update();
                RollerMotor.Update();
                LiftMotor.Update();
            END_IF;
        END_METHOD
    END_CLASS
END_NAMESPACE
