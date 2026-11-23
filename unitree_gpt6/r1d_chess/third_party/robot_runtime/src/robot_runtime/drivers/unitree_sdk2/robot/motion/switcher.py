"""Unitree motion and Teleop service startup and restoration lifecycle.

``MotionSwitcherClient`` selects the baseline motion mode or releases it for
debug control. G1 Teleop is a separate ``RobotStateClient`` service: it releases
AI itself, and MotionSwitcher reports an empty name while Teleop controls the
robot. Teleop must stop before the baseline motion mode can be restored.

MotionSwitcher operates at the service-selection layer. It does not change an
AISport FSM ID; FSM transitions are handled separately by ``AISportFSM``.
``WholeBodySdkRobotChannel`` uses this lifecycle around whole-body commands.
``LowCmdRobotChannel`` uses the released state for configured debug control.
"""

import time

import logging_mp
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient


logger_mp = logging_mp.getLogger(__name__)

MOTION_SWITCHER_BUSY_CODE = 7002
MOTION_SWITCHER_API_TIMEOUT_CODE = 3104
MOTION_SWITCHER_RPC_TIMEOUT = 2.0
MOTION_SWITCHER_CHECK_ATTEMPTS = 3
MOTION_SWITCHER_RETRY_INTERVAL = 1.0


class MotionServiceSwitcher:
    """Enter named or released control and restore a baseline service.

    Named control starts Teleop from the configured baseline through
    RobotStateClient. Debug control also accepts an already released state.
    Failed startup stops Teleop before attempting baseline restoration.

    Args:
        control_motion_mode: Optional RobotState service name, such as
            ``teleop``. ``None`` is used when control releases motion modes.
        restore_motion_mode: Baseline MotionSwitcher mode, ``ai`` for Teleop.

    Attributes:
        control_motion_mode: Configured external-control service name, if any.
        restore_motion_mode: Configured baseline service name.
        client: Initialized SDK MotionSwitcher client, or ``None`` before
            ``initialize`` and after ``shutdown``.
        robot_state_client: Service client used only for named control.
        restore_required: Whether a control-service selection still requires
            a baseline restoration attempt.

    Example:
        switcher = MotionServiceSwitcher(
            control_motion_mode="teleop",
            restore_motion_mode="ai",
        )
        switcher.initialize()
        try:
            baseline_service, baseline_state = switcher.service_state()
            switcher.select_control_service()
            # Publish commands accepted by the teleop service here.
            switcher.restore_service()
        finally:
            switcher.shutdown()
    """

    def __init__(
        self,
        *,
        restore_motion_mode: str,
        control_motion_mode: str | None = None,
    ) -> None:
        self.control_motion_mode = control_motion_mode
        self.restore_motion_mode = restore_motion_mode
        self.client: MotionSwitcherClient | None = None
        self.robot_state_client: RobotStateClient | None = None
        self.restore_required = False

    def initialize(self) -> None:
        """Create the SDK client without changing the active service.

        Repeated calls are no-ops while a client is already initialized.
        """
        if self.client is not None:
            return
        client = MotionSwitcherClient()
        client.SetTimeout(MOTION_SWITCHER_RPC_TIMEOUT)
        client.Init()
        if self.control_motion_mode is not None:
            robot_state_client = RobotStateClient()
            robot_state_client.SetTimeout(MOTION_SWITCHER_RPC_TIMEOUT)
            robot_state_client.Init()
            self.robot_state_client = robot_state_client
        self.client = client

    def shutdown(self) -> None:
        """Attempt baseline restoration and release the SDK client.

        Failed Teleop restoration retains the clients and restoration flag
        so a later call can retry instead of forgetting the active service.
        """
        if self.client is None:
            return
        self.restore_service()
        self.client = None
        self.robot_state_client = None
        self.restore_required = False

    def service_state(self) -> tuple[str, dict[str, object]]:
        """Read the active motion-service name and complete SDK response.

        Returns:
            Pair of the active service name and the complete ``CheckMode``
            response mapping.

        Raises:
            RuntimeError: The client is unavailable, ``CheckMode`` fails, or
                its response does not contain a string service name.
        """
        payload = None
        for attempt in range(1, MOTION_SWITCHER_CHECK_ATTEMPTS + 1):
            code, payload = self.require_client().CheckMode()
            if code == 0:
                break
            if (
                code != MOTION_SWITCHER_API_TIMEOUT_CODE
                or attempt == MOTION_SWITCHER_CHECK_ATTEMPTS
            ):
                raise RuntimeError(
                    f"MotionSwitcher CheckMode failed with code {code}."
                )
            logger_mp.warning(
                "MotionSwitcher CheckMode timed out with code %d; "
                "retrying (%d/%d).",
                code,
                attempt,
                MOTION_SWITCHER_CHECK_ATTEMPTS,
            )
            time.sleep(MOTION_SWITCHER_RETRY_INTERVAL)
        if not isinstance(payload, dict):
            raise RuntimeError(
                "MotionSwitcher CheckMode returned "
                f"{type(payload).__name__}, expected an object."
            )
        name = payload.get("name")
        if not isinstance(name, str):
            raise RuntimeError(
                f"MotionSwitcher service name is not a string: {name!r}."
            )
        return name, payload

    def wait_for_service(
        self,
        expected_service: str,
        timeout: float = 10.0,
    ) -> tuple[str, dict[str, object]]:
        """Poll MotionSwitcher until it reports the expected service.

        Args:
            expected_service: Service name that must become active.
            timeout: Maximum polling duration in seconds.

        Returns:
            Pair of the confirmed service name and its complete state mapping.

        Raises:
            RuntimeError: State polling fails or the expected service is not
                observed before the timeout.
        """
        deadline = time.monotonic() + timeout
        last_service = None
        last_payload = None
        while time.monotonic() < deadline:
            last_service, last_payload = self.service_state()
            if last_service == expected_service:
                return last_service, last_payload
            time.sleep(0.1)
        raise RuntimeError(
            "MotionSwitcher service check failed. "
            f"expected={expected_service!r} actual={last_service!r} "
            f"state={last_payload!r}"
        )

    def select_service(
        self,
        service: str,
        timeout: float = 10.0,
    ) -> tuple[str, dict[str, object]]:
        """Request a motion-service selection and wait for confirmation.

        Args:
            service: Target Unitree motion-service name.
            timeout: Maximum selection and confirmation duration in seconds.

        Returns:
            Pair of the confirmed service name and its complete state mapping.

        Raises:
            RuntimeError: The client is unavailable, ``SelectMode`` fails, or
                confirmation times out.
        """
        deadline = time.monotonic() + timeout
        while True:
            code, _ = self.require_client().SelectMode(service)
            if code in {0, MOTION_SWITCHER_API_TIMEOUT_CODE}:
                break
            if code != MOTION_SWITCHER_BUSY_CODE:
                raise RuntimeError(
                    f"MotionSwitcher SelectMode({service!r}) failed with "
                    f"code {code}."
                )
            actual_service, payload = self.service_state()
            if actual_service == service:
                logger_mp.info(
                    "MotionSwitcher service ready expected=%r actual=%r.",
                    service,
                    actual_service,
                )
                return actual_service, payload
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"MotionSwitcher SelectMode({service!r}) remained busy "
                    f"for {timeout:.1f}s."
                )
            time.sleep(MOTION_SWITCHER_RETRY_INTERVAL)
        actual_service, payload = self.wait_for_service(
            service,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        logger_mp.info(
            "MotionSwitcher service ready expected=%r actual=%r.",
            service,
            actual_service,
        )
        return actual_service, payload

    def wait_for_service_statuses(
        self,
        expected_statuses: dict[str, int],
        *,
        motion_mode: str | None = None,
        timeout: float = 15.0,
    ) -> None:
        """Confirm service processes and, optionally, the motion mode.

        ServiceList reports 0 for running and 1 for stopped. ServiceSwitch
        only acknowledges the request; its result does not confirm completion.
        """
        if self.robot_state_client is None:
            raise RuntimeError("RobotState client is not initialized.")
        deadline = time.monotonic() + timeout
        statuses = {}
        actual_mode = None
        code = None
        while time.monotonic() < deadline:
            code, services = self.robot_state_client.ServiceList()
            if code == 0:
                statuses = {service.name: service.status for service in services}
                missing = set(expected_statuses) - statuses.keys()
                if missing:
                    raise RuntimeError(
                        f"RobotState services not found: {sorted(missing)!r}."
                    )
                if all(
                    statuses[name] == status
                    for name, status in expected_statuses.items()
                ):
                    if motion_mode is None:
                        return
                    actual_mode, _ = self.service_state()
                    if actual_mode == motion_mode:
                        return
            elif code != MOTION_SWITCHER_API_TIMEOUT_CODE:
                raise RuntimeError(
                    f"RobotState ServiceList failed with code {code}."
                )
            time.sleep(0.1)
        statuses = {name: statuses.get(name) for name in expected_statuses}
        raise RuntimeError(
            "RobotState service transition timed out. "
            f"expected={expected_statuses!r} "
            f"actual={statuses!r} "
            f"expected_motion_mode={motion_mode!r} "
            f"actual_motion_mode={actual_mode!r} last_code={code}"
        )

    def select_control_service(self) -> None:
        """Start or reuse Teleop and confirm that it has released AI.

        Raises:
            RuntimeError: The current service is neither the baseline nor
                a confirmed running Teleop service, or selection fails.
        """
        if self.control_motion_mode is None:
            raise RuntimeError("No named control motion service is configured.")
        if self.robot_state_client is None:
            raise RuntimeError("RobotState client is not initialized.")
        actual_service, payload = self.service_state()
        if actual_service == "":
            self.wait_for_service_statuses(
                {self.control_motion_mode: 0, "ai_sport": 1},
                motion_mode="",
            )
            # Reusing a service does not create a restoration obligation.
            logger_mp.info("Reusing running Teleop service; AI is stopped.")
            return
        if actual_service != self.restore_motion_mode:
            raise RuntimeError(
                "Refusing to select control from an unexpected motion service. "
                f"expected={self.restore_motion_mode!r} "
                f"actual={actual_service!r} "
                f"state={payload!r}"
            )
        self.restore_required = True
        try:
            code = self.robot_state_client.ServiceSwitch(
                self.control_motion_mode, True
            )
            if code not in {0, MOTION_SWITCHER_API_TIMEOUT_CODE}:
                raise RuntimeError(
                    f"RobotState ServiceSwitch({self.control_motion_mode!r}, "
                    f"True) failed with code {code}."
                )
            self.wait_for_service_statuses(
                {self.control_motion_mode: 0, "ai_sport": 1},
                motion_mode="",
            )
        except Exception as exc:
            try:
                self.restore_service()
            except Exception as restore_exc:
                raise RuntimeError(
                    f"Teleop startup failed: {exc}; "
                    f"baseline restoration also failed: {restore_exc}"
                ) from exc
            raise
        logger_mp.info("Teleop service ready; AI motion service has stopped.")

    def release_control_service(self, timeout: float = 10.0) -> None:
        """Release active motion services and confirm the debug state.

        Args:
            timeout: Maximum release and confirmation duration in seconds.

        Raises:
            RuntimeError: The current service is neither
                ``restore_motion_mode`` nor the released debug state,
                releasing a service fails, or the released state is not
                confirmed before the timeout.
        """
        actual_service, payload = self.service_state()
        if not actual_service:
            # Unitree LowCmd examples also proceed when CheckMode already
            # reports no active service; remember to restore the configured
            # baseline when this control session ends.
            self.restore_required = True
            logger_mp.info(
                "MotionSwitcher already released for debug control."
            )
            return
        if actual_service != self.restore_motion_mode:
            raise RuntimeError(
                "Refusing to release an unexpected motion service. "
                f"expected={self.restore_motion_mode!r} "
                f"actual={actual_service!r} "
                f"state={payload!r}"
            )

        self.restore_required = True
        deadline = time.monotonic() + timeout
        try:
            while actual_service:
                code, _ = self.require_client().ReleaseMode()
                if code not in {0, MOTION_SWITCHER_BUSY_CODE}:
                    raise RuntimeError(
                        f"MotionSwitcher ReleaseMode failed with code {code}."
                    )
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "MotionSwitcher release could not be confirmed. "
                        f"actual={actual_service!r} state={payload!r} "
                        f"last_code={code}"
                    )
                time.sleep(
                    MOTION_SWITCHER_RETRY_INTERVAL
                    if code == MOTION_SWITCHER_BUSY_CODE
                    else 0.1
                )
                actual_service, payload = self.service_state()
        except Exception:
            self.restore_service()
            raise
        logger_mp.info("MotionSwitcher services released for debug control.")

    def restore_service(self) -> None:
        """Stop Teleop before restoring the configured baseline mode.

        The method is a no-op when restoration is not required. After a
        selection error, the reported service is checked once because an RPC
        timeout may still have completed the transition. Teleop restoration
        failures raise and leave ``restore_required`` set for a later retry.
        Released debug control retains its best-effort restoration behavior.
        """
        if not self.restore_required:
            return
        if self.control_motion_mode is not None:
            if self.robot_state_client is None:
                raise RuntimeError("RobotState client is not initialized.")
            code = self.robot_state_client.ServiceSwitch(
                self.control_motion_mode, False
            )
            if code not in {0, MOTION_SWITCHER_API_TIMEOUT_CODE}:
                raise RuntimeError(
                    f"RobotState ServiceSwitch({self.control_motion_mode!r}, "
                    f"False) failed with code {code}."
                )
            self.wait_for_service_statuses({self.control_motion_mode: 1})
            deadline = time.monotonic() + 15.0
            self.select_service(self.restore_motion_mode)
            self.wait_for_service_statuses(
                {self.control_motion_mode: 1, "ai_sport": 0},
                motion_mode=self.restore_motion_mode,
                timeout=max(0.1, deadline - time.monotonic()),
            )
            self.restore_required = False
            return
        try:
            self.select_service(self.restore_motion_mode)
        except Exception as exc:
            try:
                actual_service, payload = self.service_state()
            except Exception as state_exc:
                logger_mp.warning(
                    "MotionSwitcher transition to %r could not be "
                    "confirmed: %s; state check failed: %s",
                    self.restore_motion_mode,
                    exc,
                    state_exc,
                )
                return
            if actual_service == self.restore_motion_mode:
                logger_mp.info(
                    "MotionSwitcher service %r confirmed after selection "
                    "reported an error: %s",
                    self.restore_motion_mode,
                    exc,
                )
                self.restore_required = False
                return
            logger_mp.warning(
                "MotionSwitcher transition to %r could not be confirmed: "
                "%s; actual=%r state=%r",
                self.restore_motion_mode,
                exc,
                actual_service,
                payload,
            )
            return
        self.restore_required = False

    def require_client(self) -> MotionSwitcherClient:
        """Return the initialized MotionSwitcher client.

        Returns:
            Active SDK MotionSwitcher client.

        Raises:
            RuntimeError: ``initialize`` has not completed or ``shutdown``
                has already released the client.
        """
        if self.client is None:
            raise RuntimeError("MotionSwitcher client is not initialized.")
        return self.client
