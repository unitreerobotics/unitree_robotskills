"""Unitree AISport RPC access and confirmed FSM transitions.

``AISportClient`` wraps the ``sport`` service APIs used to read and change
the active finite-state-machine (FSM) ID. ``AISportFSM`` owns that client and
adds a control lifecycle: verify a configured baseline FSM, enter the control
FSM, and restore the baseline when control ends.

This module changes FSMs inside AISport. It does not select named Unitree
motion services; service selection belongs to ``MotionServiceSwitcher``.
``Pose82dRobotChannel`` uses ``AISportFSM`` around pose-command publication.
"""

import json
import time

import logging_mp
from unitree_sdk2py.rpc.client import Client


FSM_SERVICE_NAME = "sport"
FSM_API_VERSION = "1.0.0.0"

ROBOT_API_ID_GET_FSM_ID = 7001
ROBOT_API_ID_GET_FSM_MODE = 7002
ROBOT_API_ID_GET_ARM_SDK_STATUS = 7007
ROBOT_API_ID_SET_FSM_ID = 7101
ROBOT_API_ID_SET_ARM_SDK_STATUS = 7109


logger_mp = logging_mp.getLogger(__name__)


class AISportClient(Client):
    """Call the FSM APIs exposed by the Unitree AISport service.

    ``Init`` must be called after the Unitree SDK2 channel factory is
    initialized and before any query or transition. ``set_fsm_id`` submits
    one transition request but does not wait for the requested FSM to become
    active; ``AISportFSM.switch_fsm`` provides that confirmed operation.

    Example:
        client = AISportClient()
        client.SetTimeout(10.0)
        client.Init()

        current_fsm_id = client.get_fsm_id()
        current_fsm_mode = client.get_fsm_mode()
        client.set_fsm_id(505)
    """

    def __init__(self) -> None:
        super().__init__(FSM_SERVICE_NAME, False)

    def Init(self) -> None:
        """Set the AISport API version and register the required FSM APIs."""
        self._SetApiVerson(FSM_API_VERSION)
        self._RegistApi(ROBOT_API_ID_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_GET_ARM_SDK_STATUS, 0)
        self._RegistApi(ROBOT_API_ID_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_SET_ARM_SDK_STATUS, 0)

    def get_fsm_id(self) -> int:
        """Return the active FSM identifier.

        Returns:
            Active FSM ID reported by AISport.

        Raises:
            RuntimeError: The RPC fails, returns malformed JSON, or provides a
                non-numeric FSM ID.
        """
        value = self.call_data(ROBOT_API_ID_GET_FSM_ID, "{}")
        try:
            return int(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"FSM id is not numeric: {value!r}.") from exc

    def get_fsm_mode(self) -> object:
        """Return the active FSM mode reported by AISport.

        Returns:
            Unmodified ``data`` value from the FSM-mode response.

        Raises:
            RuntimeError: The RPC fails or returns an invalid response.
        """
        return self.call_data(ROBOT_API_ID_GET_FSM_MODE, "{}")

    def set_fsm_id(self, fsm_id: int) -> None:
        """Submit an FSM transition request without polling its result.

        Args:
            fsm_id: Target AISport FSM identifier.

        Raises:
            RuntimeError: The transition RPC fails or returns an invalid
                response.
        """
        parameter = json.dumps({"data": fsm_id})
        self.call_data(ROBOT_API_ID_SET_FSM_ID, parameter)

    def get_arm_sdk_status(self) -> bool:
        """Read the H2 external Arm SDK enable flag."""
        value = self.call_data(ROBOT_API_ID_GET_ARM_SDK_STATUS, "{}")
        if not isinstance(value, bool):
            raise RuntimeError(f"Arm SDK status is not boolean: {value!r}.")
        return value

    def set_arm_sdk_status(self, enabled: bool) -> None:
        """Set the H2 external Arm SDK flag and confirm it by reading back."""
        self.call_data(
            ROBOT_API_ID_SET_ARM_SDK_STATUS, json.dumps({"data": enabled}),
        )
        deadline = time.monotonic() + 3.0
        actual = None
        while time.monotonic() < deadline:
            actual = self.get_arm_sdk_status()
            if actual == enabled:
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"Arm SDK check failed. expected={enabled} actual={actual}"
        )

    def call_data(self, api_id: int, parameter: str) -> object:
        """Call a registered AISport API and extract its JSON ``data`` value.

        Args:
            api_id: Registered Unitree RPC API identifier.
            parameter: JSON request passed to the SDK client.

        Returns:
            Response object's ``data`` value, or ``None`` when it is absent.

        Raises:
            RuntimeError: The RPC reports an error, returns malformed JSON, or
                returns a JSON value other than an object.
        """
        code, raw = self._Call(api_id, parameter)
        if code != 0:
            raise RuntimeError(f"FSM RPC {api_id} failed with code {code}.")
        try:
            payload = json.loads(raw) if raw else {}
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"FSM RPC {api_id} returned invalid JSON: {raw!r}."
            ) from exc
        if not isinstance(payload, dict):
            raise RuntimeError(
                f"FSM RPC {api_id} returned {type(payload).__name__}, "
                "expected an object."
            )
        return payload.get("data")


class AISportFSM:
    """Manage one AISport control FSM and its configured baseline FSM.

    Entering control is allowed only when AISport currently reports
    ``restore_fsm_id``. Successful entry marks restoration as required.
    Failed entry also attempts restoration before propagating the original
    transition error.

    Args:
        control_fsm_id: FSM selected while external commands are active.
        restore_fsm_id: Required baseline FSM and restoration target.

    Attributes:
        control_fsm_id: Configured external-control FSM ID.
        restore_fsm_id: Configured baseline FSM ID.
        client: Initialized AISport RPC client, or ``None`` before
            ``initialize`` and after ``shutdown``.
        restore_required: Whether a control transition still requires a
            baseline restoration attempt.

    Example:
        fsm = AISportFSM(
            control_fsm_id=505,
            restore_fsm_id=801,
        )
        fsm.initialize()
        try:
            baseline_id, baseline_mode = fsm.fsm_state()
            fsm.enter_control_fsm()
            # Publish commands accepted by FSM 505 here.
            fsm.restore_fsm()
        finally:
            fsm.shutdown()
    """

    def __init__(
        self,
        *,
        control_fsm_id: int,
        restore_fsm_id: int,
    ) -> None:
        self.control_fsm_id = control_fsm_id
        self.restore_fsm_id = restore_fsm_id
        self.client: AISportClient | None = None
        self.restore_required = False

    def initialize(self) -> None:
        """Create the RPC client without changing the active FSM.

        Repeated calls are no-ops while a client is already initialized.
        """
        if self.client is not None:
            return
        client = AISportClient()
        client.SetTimeout(10.0)
        client.Init()
        self.client = client

    def shutdown(self) -> None:
        """Attempt baseline restoration and release the RPC client.

        Restoration failures are logged by ``restore_fsm``. The local client
        and restoration flag are cleared regardless of that outcome.
        """
        if self.client is None:
            return
        try:
            self.restore_fsm()
        finally:
            self.client = None
            self.restore_required = False

    def fsm_state(self) -> tuple[int, object]:
        """Read the current FSM identifier and mode.

        Returns:
            Pair of active FSM ID and the unmodified FSM-mode value.

        Raises:
            RuntimeError: The client is not initialized or either RPC fails.
        """
        client = self.require_client()
        return client.get_fsm_id(), client.get_fsm_mode()

    def wait_for_fsm(
        self,
        expected_fsm_id: int,
        timeout: float = 3.0,
    ) -> tuple[int, object]:
        """Poll AISport until it reports the expected FSM.

        Args:
            expected_fsm_id: FSM identifier that must become active.
            timeout: Maximum polling duration in seconds.

        Returns:
            Pair of the confirmed FSM ID and its reported mode.

        Raises:
            RuntimeError: State polling fails or the expected FSM is not
                observed before the timeout.
        """
        deadline = time.monotonic() + timeout
        last_fsm_id = None
        last_fsm_mode = None
        while time.monotonic() < deadline:
            last_fsm_id, last_fsm_mode = self.fsm_state()
            if last_fsm_id == expected_fsm_id:
                return last_fsm_id, last_fsm_mode
            time.sleep(0.05)
        raise RuntimeError(
            "FSM check failed. "
            f"expected={expected_fsm_id} actual={last_fsm_id} "
            f"mode={last_fsm_mode}"
        )

    def switch_fsm(
        self,
        fsm_id: int,
    ) -> tuple[int, object]:
        """Request an FSM transition and wait for its confirmation.

        Args:
            fsm_id: Target AISport FSM identifier.

        Returns:
            Pair of the confirmed FSM ID and its reported mode.

        Raises:
            RuntimeError: The client is unavailable, the request fails, or
                confirmation times out.
        """
        client = self.require_client()
        client.SetTimeout(3.0)
        client.set_fsm_id(fsm_id)
        actual_fsm_id, fsm_mode = self.wait_for_fsm(fsm_id)
        logger_mp.info(
            "FSM ready expected=%d actual=%d mode=%s.",
            fsm_id,
            actual_fsm_id,
            fsm_mode,
        )
        return actual_fsm_id, fsm_mode

    def enter_control_fsm(self) -> None:
        """Verify the baseline FSM and enter the configured control FSM.

        Raises:
            RuntimeError: The current FSM is not ``restore_fsm_id`` or the
                control transition cannot be completed.
        """
        actual_fsm_id, fsm_mode = self.fsm_state()
        if actual_fsm_id != self.restore_fsm_id:
            raise RuntimeError(
                "Refusing to enter control from an unexpected FSM. "
                f"expected={self.restore_fsm_id} actual={actual_fsm_id} "
                f"mode={fsm_mode}"
            )
        try:
            self.switch_fsm(self.control_fsm_id)
        except Exception:
            self.restore_required = True
            self.restore_fsm()
            raise
        self.restore_required = True

    def restore_fsm(self) -> None:
        """Best-effort restore the configured baseline FSM.

        The method is a no-op when restoration is not required. Transition
        failures are logged and leave ``restore_required`` set so a later
        call may retry.
        """
        if not self.restore_required:
            return
        try:
            self.switch_fsm(self.restore_fsm_id)
        except Exception as exc:
            logger_mp.warning(
                "FSM transition to %d could not be confirmed: %s",
                self.restore_fsm_id,
                exc,
            )
            return
        self.restore_required = False

    def require_client(self) -> AISportClient:
        """Return the initialized AISport client.

        Returns:
            Active AISport RPC client.

        Raises:
            RuntimeError: ``initialize`` has not completed or ``shutdown``
                has already released the client.
        """
        if self.client is None:
            raise RuntimeError("AISport client is not initialized.")
        return self.client
