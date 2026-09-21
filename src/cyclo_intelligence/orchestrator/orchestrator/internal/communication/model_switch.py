"""Manual, same-episode SH5 ViTacFormer handoff used by the inference UI."""

from copy import deepcopy
from dataclasses import dataclass, field
import threading

from interfaces.msg import InferenceStatus
from interfaces.srv import SendCommand

from .container_service_client import ContainerServiceClient
from .inference_mode import publish_to_robot_override_from_task_info


SWITCH_INFERENCE_MODEL_COMMAND = 26
PRELOAD_INFERENCE_MODEL_COMMAND = 27
CLEAR_PRELOADED_MODEL_COMMAND = 28
SWITCH_PRELOADED_MODEL_COMMAND = 29


@dataclass
class ModelSwitchOperation:
    cancelled: threading.Event = field(default_factory=threading.Event)
    activation_lock: threading.Lock = field(default_factory=threading.Lock)
    paused: bool = False
    resumed: bool = False
    preparing: bool = False


def intercept_model_switch_command(node, request, response):
    """Keep other lifecycle calls from racing a switch; Stop cancels resume."""
    if (getattr(node, '_model_switch_needs_retry', False)
            and request.command in {
                SendCommand.Request.START_INFERENCE,
                SendCommand.Request.RESUME_INFERENCE,
                25,
            }):
        response.success = False
        response.message = 'Model switch was not confirmed; retry switching or Clear and reload'
        return response
    if getattr(node, '_model_switch_operation', None) is None:
        return None
    with node._state_lock:
        operation = getattr(node, '_model_switch_operation', None)
        client = node.container_service_client
    if operation is None:
        return None
    if request.command == SendCommand.Request.STOP_INFERENCE:
        operation.cancelled.set()
        # Serialize with the initial PAUSE and the final RESUME. Once Stop
        # acknowledges, this operation can never activate robot output again.
        with operation.activation_lock:
            if operation.resumed and client is not None:
                result = client.inference_command(ContainerServiceClient.CMD_PAUSE)
                response.success = result.success
                response.message = result.message
                if result.success:
                    operation.resumed = False
                    node._publish_inference_phase(InferenceStatus.PAUSED)
            else:
                response.success = operation.paused
                response.message = (
                    'Model preparation does not start inference'
                    if operation.preparing else
                    'Model switch resume cancelled; inference remains paused'
                    if operation.paused else 'Pause has not been acknowledged'
                )
        return response
    blocked = {
        SendCommand.Request.START_INFERENCE,
        SendCommand.Request.RESUME_INFERENCE,
        SendCommand.Request.FINISH,
        SendCommand.Request.CANCEL,
        SendCommand.Request.UPDATE_INSTRUCTION,
        25,  # PREPARE_NEXT_CYCLE
        SWITCH_INFERENCE_MODEL_COMMAND,
        PRELOAD_INFERENCE_MODEL_COMMAND,
        CLEAR_PRELOADED_MODEL_COMMAND,
        SWITCH_PRELOADED_MODEL_COMMAND,
    }
    if request.command in blocked:
        response.success = False
        response.message = ('Model preparation in progress; wait for it to finish'
                            if operation.preparing else
                            'Model switch in progress; Stop cancels automatic resume')
        return response
    return None


def switch_inference_model(node, request, response):
    """Pause -> replace weights -> checked resume, retaining the loaded mode.

    A long weight load never shares a deadline with the motion activation.
    RESUME is a separate bounded request with the runtime's normal preflight.
    Recording, episode calibration and the task's initial pose are untouched.
    """
    response.success = False
    response.loaded_policy_path = node._loaded_inference_policy_path
    response.inference_phase = getattr(
        node, '_inference_phase', InferenceStatus.PAUSED
    )
    if not node._inference_lifecycle_lock.acquire(blocking=False):
        response.message = 'Another inference lifecycle operation is in progress'
        return response
    operation = ModelSwitchOperation()
    switch_completed = False
    try:
        task_info = request.task_info
        target_path = node._normalize_policy_path(task_info.policy_path)
        with node._state_lock:
            client = node.container_service_client
            loaded_mode = node._loaded_inference_publish_to_robot
            loaded_path = node._loaded_inference_policy_path
        if (node.robot_type != 'ffw_sh5_rev1' or client is None
                or client._service_prefix != '/vitacformer'):
            raise ValueError('Manual model switching requires SH5 ViTacFormer inference')
        if not node.on_inference or not loaded_path:
            raise ValueError('Start the first model before switching')
        if getattr(node, '_inference_cycle_prepared', False):
            raise ValueError('Cycle Home is pending; start the cycle before switching')
        if not target_path or target_path == loaded_path:
            raise ValueError('Select a different next model path')
        if node._determine_service_prefix(task_info) != '/vitacformer':
            raise ValueError('The next model must use the ViTacFormer backend')
        requested_mode = publish_to_robot_override_from_task_info(task_info)
        if requested_mode is not None and requested_mode != loaded_mode:
            raise ValueError('Model switching cannot change the deploy mode')

        # Install while holding activation_lock so a simultaneous Stop cannot
        # return before the first pause acknowledgement.
        with operation.activation_lock:
            with node._state_lock:
                node._model_switch_operation = operation
            result = client.inference_command(ContainerServiceClient.CMD_PAUSE)
            if not result.success:
                raise RuntimeError(result.message or 'Could not pause current model')
            operation.paused = True
            response.inference_phase = InferenceStatus.PAUSED
        node._publish_inference_phase(InferenceStatus.LOADING)
        result = client.inference_command(
            (ContainerServiceClient.CMD_SWITCH_PRELOADED
             if request.command == SWITCH_PRELOADED_MODEL_COMMAND
             else ContainerServiceClient.CMD_SWITCH_POLICY),
            model_path=target_path,
            robot_type=node.robot_type,
        )
        if not result.success:
            raise RuntimeError(result.message or 'Next model load failed')
        switch_completed = True

        with node._state_lock:
            if node.container_service_client is not client:
                raise RuntimeError('Inference session changed during model switch')
            node._loaded_inference_policy_path = target_path
            node._inference_cycle_prepared = False
            node._model_switch_needs_retry = False
        response.loaded_policy_path = target_path
        # Preserve task instructions and runtime knobs from the loaded episode.
        cached = deepcopy(getattr(node, '_prepared_inference_task_info', None) or task_info)
        cached.policy_path = target_path
        cached.task_type = 'inference'
        cached.service_type = 'vitacformer'
        node._cache_ui_task_info(cached, 'SWITCH_INFERENCE_MODEL')

        with operation.activation_lock:
            with node._state_lock:
                if node.container_service_client is not client:
                    raise RuntimeError('Inference session changed during model switch')
            if operation.cancelled.is_set():
                response.success = True
                response.message = 'Next model loaded; automatic resume cancelled'
            else:
                result = client.inference_command(
                    ContainerServiceClient.CMD_RESUME,
                    publish_to_robot=loaded_mode,
                )
                if not result.success:
                    raise RuntimeError(
                        'Next model loaded but remains paused: ' + result.message
                    )
                operation.resumed = True
                response.inference_phase = InferenceStatus.INFERENCING
                response.success = True
                response.message = 'Next model is running'
            node._publish_inference_phase(response.inference_phase)
    except Exception as exc:
        response.message = str(exc)
        if operation.paused:
            if not switch_completed:
                # The outer service can time out before Main/Engine finish.
                # A later ordinary Resume must not run an unconfirmed policy.
                node._model_switch_needs_retry = True
            response.inference_phase = InferenceStatus.PAUSED
            node._publish_inference_phase(InferenceStatus.PAUSED, error=str(exc))
    finally:
        # A concurrent Stop may have paused just after RESUME completed.
        if operation.cancelled.is_set() and operation.paused and not operation.resumed:
            response.inference_phase = InferenceStatus.PAUSED
        with node._state_lock:
            if getattr(node, '_model_switch_operation', None) is operation:
                node._model_switch_operation = None
        node._inference_lifecycle_lock.release()
    return response


def prepare_inference_model(node, request, response):
    """Prepare/release a spare while idle; never LOAD/START a robot session."""
    response.success = False
    if not node._inference_lifecycle_lock.acquire(blocking=False):
        response.message = 'Another inference lifecycle operation is in progress'
        return response
    temporary_client = None
    operation = ModelSwitchOperation(paused=True, preparing=True)
    try:
        task_info = request.task_info
        if (node.robot_type != 'ffw_sh5_rev1'
                or node._determine_service_prefix(task_info) != '/vitacformer'):
            raise ValueError('Preloading requires SH5 ViTacFormer')
        phase = getattr(node, '_inference_phase', InferenceStatus.READY)
        if phase not in {InferenceStatus.READY, InferenceStatus.PAUSED}:
            raise ValueError('Preload before Start, or press Stop to pause inference first')
        target_path = node._normalize_policy_path(task_info.policy_path)
        clearing = request.command == CLEAR_PRELOADED_MODEL_COMMAND
        if not clearing and (not target_path or target_path == node._loaded_inference_policy_path):
            raise ValueError('Select a different next model path')
        with node._state_lock:
            client = node.container_service_client
            node._model_switch_operation = operation
        if client is not None and client._service_prefix != '/vitacformer':
            raise ValueError('Clear the current backend before preloading ViTacFormer')
        if client is None:
            # Do not install this as the active session client: a subsequent
            # first-model START would tear it down and UNLOAD the spare.
            temporary_client = ContainerServiceClient(
                node=node, service_prefix='/vitacformer',
                callback_group=node._client_cb_group,
            )
            client = temporary_client
            client.connect()
        result = client.inference_command(
            ContainerServiceClient.CMD_CLEAR_PRELOAD if clearing else ContainerServiceClient.CMD_PRELOAD_POLICY,
            model_path=target_path, robot_type=node.robot_type,
        )
        response.success = result.success
        response.message = result.message
    except Exception as exc:
        response.message = str(exc)
    finally:
        if temporary_client is not None:
            temporary_client.disconnect()
        with node._state_lock:
            if getattr(node, '_model_switch_operation', None) is operation:
                node._model_switch_operation = None
        node._inference_lifecycle_lock.release()
    return response
