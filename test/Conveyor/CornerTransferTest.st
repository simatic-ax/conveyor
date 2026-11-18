USING Simatic.Ax.Conveyor.Mocks;
USING Simatic.Ax.Conveyor;
USING AxUnit.Assert;
USING Simatic.Ax.AutomationBase;
USING MotorTests.Mocks;

NAMESPACE Simatic.Ax.Conveyor
    /// Tests for the CornerTransfer sequence in automatic mode
    {TestFixture}
    CLASS CornerTransferTest
        VAR
            ct : CornerTransfer;
            ctStateless : CornerTransfer;
            mode : MockOperatingMode;
            modeStateless : MockOperatingMode;
            rel : MockIsReleased_TRUE;
            autoStart : MockAutoStartup;
            autoStartStateless : MockAutoStartup;
            stopSensor : MockBinSignal;
            rollerEndSensor : MockBinSignal;
            liftUpSensor : MockBinSignal;
            sensorStateless : MockBinSignal;
            motor : MockMotor;
            rollerMotor : MockMotor;
            liftMotor : MockMotor;
            motorStateless : MockMotor;
            next : MockingConveyor;
            nextStateless : MockingConveyor;
            prev : MockingConveyor;
            resetMgr : MockResetManager;
            resetMgrStateless : MockResetManager;
        END_VAR

        {TestSetup}
        METHOD PUBLIC Setup
            ct := ctStateless;
            mode := modeStateless;
            autoStart := autoStartStateless;
            stopSensor := sensorStateless;
            rollerEndSensor := sensorStateless;
            liftUpSensor := sensorStateless;
            motor := motorStateless;
            rollerMotor := motorStateless;
            liftMotor := motorStateless;
            next := nextStateless;
            prev := nextStateless;
            resetMgr := resetMgrStateless;

            mode.mode := OperatingModes#Automatic;
            autoStart.startupStatus := StartUpStatus#AutoStartedUp;
            next.RTR := ReceiveState#Idle;

            ct.Mode := mode;
            ct.ExternalRelease := rel;
            ct.ExternalAutoStartup := autoStart;
            ct.StopSensor := stopSensor;
            ct.RollerEndSensor := rollerEndSensor;
            ct.LiftUpSensor := liftUpSensor;
            ct.Motor := motor;
            ct.RollerMotor := rollerMotor;
            ct.LiftMotor := liftMotor;
            ct.NextConveyor := next;
            ct.ResetCommand := resetMgr;
        END_METHOD

        /// Drives the sequence until the goods are lifted and wait for the next conveyor
        METHOD PRIVATE RunToReadyToDeliver
            stopSensor.QValue := TRUE;
            ct.RunCyclic();
            ct.RunCyclic();
            rollerEndSensor.QValue := TRUE;
            ct.RunCyclic();
            ct.RunCyclic();
            liftUpSensor.QValue := TRUE;
            ct.RunCyclic();
            ct.RunCyclic();
        END_METHOD

        {Test}
        METHOD PUBLIC Missing_sensor_raises_ConfigError
            ct.LiftUpSensor := NULL;

            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetError() = ErrorState#ConfigError);
            Equal(expected := FALSE, actual := rollerMotor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Empty_and_started_up_is_ReadyToReceive
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.ReceiveStatus() = ReceiveState#ReadyToReceive);
            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Idle);
            Equal(expected := FALSE, actual := ct.IsOccupied());
        END_METHOD

        {Test}
        METHOD PUBLIC Goods_at_StopSensor_start_roller_motor
            stopSensor.QValue := TRUE;

            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Receiving);
            Equal(expected := TRUE, actual := ct.ReceiveStatus() = ReceiveState#Receiving);
            Equal(expected := TRUE, actual := rollerMotor.IsRunning());
            Equal(expected := FALSE, actual := motor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Lifted_goods_wait_for_next_conveyor
            THIS.RunToReadyToDeliver();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#ReadyToDeliver);
            Equal(expected := TRUE, actual := ct.DeliverStatus() = DeliverState#ReadyToDeliver);
            Equal(expected := FALSE, actual := rollerMotor.IsRunning());
            Equal(expected := TRUE, actual := liftMotor.IsRunning());
            Equal(expected := FALSE, actual := motor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Next_ReadyToReceive_starts_delivering
            THIS.RunToReadyToDeliver();
            next.RTR := ReceiveState#ReadyToReceive;

            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Delivering);
            Equal(expected := TRUE, actual := ct.DeliverStatus() = DeliverState#Delivering);
            Equal(expected := TRUE, actual := motor.IsRunning());
            Equal(expected := TRUE, actual := liftMotor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Goods_removed_while_ReadyToDeliver_lowers_lift_to_Idle
            THIS.RunToReadyToDeliver();
            stopSensor.QValue := FALSE;
            rollerEndSensor.QValue := FALSE;

            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#LoweringDown);
            Equal(expected := FALSE, actual := ct.IsOccupied());

            liftUpSensor.QValue := FALSE;
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Idle);
            Equal(expected := FALSE, actual := liftMotor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Lift_drop_while_ReadyToDeliver_lifts_again
            THIS.RunToReadyToDeliver();
            liftUpSensor.QValue := FALSE;

            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#LiftingUp);
            Equal(expected := TRUE, actual := liftMotor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC TriggerStartInAutomatic_runs_roller_motor
            ct.RunCyclic();

            ct.TriggerStartInAutomatic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Idle);
            Equal(expected := TRUE, actual := rollerMotor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Manual_mode_delivers_without_ready_next_conveyor
            mode.mode := OperatingModes#Manual;

            Equal(expected := TRUE, actual := ct.StartManual());
            THIS.RunToReadyToDeliver();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Delivering);
            Equal(expected := TRUE, actual := ct.Delivering());
            Equal(expected := TRUE, actual := motor.IsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Missing_motors_use_NullMotor_without_error
            ct.Motor := NULL;
            ct.RollerMotor := NULL;
            ct.LiftMotor := NULL;

            ct.RunCyclic();

            Equal(expected := FALSE, actual := ct.GetError() = ErrorState#ConfigError);
            Equal(expected := TRUE, actual := ct.ReceiveStatus() = ReceiveState#ReadyToReceive);
            Equal(expected := FALSE, actual := ct.ConveyorIsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Missing_release_raises_ConfigError
            ct.ExternalRelease := NULL;
            stopSensor.QValue := TRUE;

            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetError() = ErrorState#ConfigError);
            Equal(expected := TRUE, actual := ct.ReceiveStatus() = ReceiveState#Idle);
            Equal(expected := FALSE, actual := ct.IsOccupied());
            Equal(expected := FALSE, actual := ct.StopSensorStatus());
        END_METHOD

        {Test}
        METHOD PUBLIC Prev_delivering_starts_roller_motor_in_Idle
            prev.RTD := DeliverState#ReadyToDeliver;
            ct.PrevConveyor := prev;

            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Idle);
            Equal(expected := TRUE, actual := ct.RollerIsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC TriggerStartInAutomatic_is_ignored_when_occupied
            ct.RunCyclic();
            stopSensor.QValue := TRUE;

            ct.TriggerStartInAutomatic();
            stopSensor.QValue := FALSE;
            ct.RunCyclic();

            Equal(expected := FALSE, actual := ct.RollerIsRunning());
        END_METHOD

        {Test}
        METHOD PUBLIC Status_getters_in_ReadyToDeliver
            THIS.RunToReadyToDeliver();

            Equal(expected := TRUE, actual := ct.StopSensorStatus());
            Equal(expected := TRUE, actual := ct.IsOccupied());
            Equal(expected := TRUE, actual := ct.LiftIsRunning());
            Equal(expected := FALSE, actual := ct.RollerIsRunning());
            Equal(expected := FALSE, actual := ct.ConveyorIsRunning());
            Equal(expected := 0.0, actual := ct.GetMotorSpeed());
            Equal(expected := FALSE, actual := ct.Delivering());
        END_METHOD

        {Test}
        METHOD PUBLIC Delivering_runs_transverse_motor_with_automatic_speed
            THIS.RunToReadyToDeliver();
            next.RTR := ReceiveState#ReadyToReceive;
            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.ConveyorIsRunning());
            Equal(expected := 1.5, actual := ct.GetMotorSpeed());
        END_METHOD

        {Test}
        METHOD PUBLIC Lift_drop_while_Delivering_lifts_again
            THIS.RunToReadyToDeliver();
            next.RTR := ReceiveState#ReadyToReceive;
            ct.RunCyclic();
            liftUpSensor.QValue := FALSE;

            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#LiftingUp);
        END_METHOD

        {Test}
        METHOD PUBLIC StartManual_in_automatic_returns_false
            Equal(expected := FALSE, actual := ct.StartManual());
        END_METHOD

        {Test}
        METHOD PUBLIC Manual_mode_runs_roller_with_manual_speed
            mode.mode := OperatingModes#Manual;
            stopSensor.QValue := TRUE;
            ct.StartManual();

            ct.RunCyclic();
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Receiving);
            Equal(expected := 0.5, actual := rollerMotor.GetCurrentSpeed());
        END_METHOD

        {Test}
        METHOD PUBLIC Stop_in_manual_mode_stops_roller_motor
            mode.mode := OperatingModes#Manual;
            stopSensor.QValue := TRUE;
            ct.StartManual();
            ct.RunCyclic();
            ct.RunCyclic();

            ct.Stop();
            ct.RunCyclic();

            Equal(expected := FALSE, actual := ct.RollerIsRunning());
            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#Receiving);
        END_METHOD

        {Test}
        METHOD PUBLIC Step_timeout_raises_error_and_stops_motors
            stopSensor.QValue := TRUE;
            ct.RunCyclic();
            ct.RunCyclic();
            AxUnit.Mocking.Mock('System.Timer.OnDelay', 'Simatic.Ax.Mocks.OnDelayMock_true');

            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetError() = ErrorState#OtherError);
            Equal(expected := FALSE, actual := ct.RollerIsRunning());
            Equal(expected := TRUE, actual := ct.ReceiveStatus() = ReceiveState#Idle);
            Equal(expected := TRUE, actual := ct.DeliverStatus() = DeliverState#Idle);
        END_METHOD

        {Test}
        METHOD PUBLIC Reset_with_lifted_goods_resumes_in_ReadyToDeliver
            ct.RunCyclic();
            stopSensor.QValue := TRUE;
            liftUpSensor.QValue := TRUE;
            ct.SetError(errorState := WORD#16#8001, errorText := 'Test error');

            resetMgr.ResetCommandActive := TRUE;
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#ReadyToDeliver);
        END_METHOD

        {Test}
        METHOD PUBLIC Reset_with_goods_at_roller_end_resumes_in_LiftingUp
            ct.RunCyclic();
            rollerEndSensor.QValue := TRUE;
            ct.SetError(errorState := WORD#16#8001, errorText := 'Test error');

            resetMgr.ResetCommandActive := TRUE;
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#LiftingUp);
        END_METHOD

        {Test}
        METHOD PUBLIC Reset_with_lift_up_and_no_goods_resumes_in_LoweringDown
            ct.RunCyclic();
            liftUpSensor.QValue := TRUE;
            ct.SetError(errorState := WORD#16#8001, errorText := 'Test error');

            resetMgr.ResetCommandActive := TRUE;
            ct.RunCyclic();

            Equal(expected := TRUE, actual := ct.GetState() = CornerTransferState#LoweringDown);
        END_METHOD
    END_CLASS
END_NAMESPACE
