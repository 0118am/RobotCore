"""Native TensorRT policy runner for NVIDIA edge computers."""

from __future__ import annotations

import ctypes
from pathlib import Path

from .base import PolicyRunner
from ..t60_observation import OBSERVATION_LAYOUT, T60ObservationState


class _CudaRuntime:
    host_to_device = 1
    device_to_host = 2

    def __init__(self):
        self.library = ctypes.CDLL("libcudart.so")
        self.library.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.library.cudaMalloc.restype = ctypes.c_int
        self.library.cudaFree.argtypes = [ctypes.c_void_p]
        self.library.cudaFree.restype = ctypes.c_int
        self.library.cudaMemcpy.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
        ]
        self.library.cudaMemcpy.restype = ctypes.c_int
        self.library.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.library.cudaStreamCreate.restype = ctypes.c_int
        self.library.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamSynchronize.restype = ctypes.c_int
        self.library.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamDestroy.restype = ctypes.c_int
        self.library.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.library.cudaGetErrorString.restype = ctypes.c_char_p

    def check(self, result, operation):
        if result == 0:
            return
        description = self.library.cudaGetErrorString(result)
        message = description.decode("utf-8") if description else "unknown CUDA error"
        raise RuntimeError(f"{operation} failed: {message} ({result})")

    def allocate(self, size):
        pointer = ctypes.c_void_p()
        self.check(
            self.library.cudaMalloc(ctypes.byref(pointer), int(size)), "cudaMalloc"
        )
        return pointer

    def free(self, pointer):
        if pointer is not None and pointer.value:
            self.check(self.library.cudaFree(pointer), "cudaFree")

    def create_stream(self):
        stream = ctypes.c_void_p()
        self.check(self.library.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def destroy_stream(self, stream):
        if stream is not None and stream.value:
            self.check(self.library.cudaStreamDestroy(stream), "cudaStreamDestroy")

    def copy(self, destination, source, size, direction):
        self.check(
            self.library.cudaMemcpy(destination, source, int(size), direction),
            "cudaMemcpy",
        )

    def synchronize(self, stream):
        self.check(self.library.cudaStreamSynchronize(stream), "cudaStreamSynchronize")


class TensorRtRunner(PolicyRunner):
    """Build and execute one static-shape ONNX actor with TensorRT 10."""

    def __init__(self, manifest):
        super().__init__(manifest)
        try:
            import numpy as np
            import tensorrt as trt
        except Exception as exc:
            raise RuntimeError(
                "TensorRT policy requested but numpy/tensorrt is unavailable"
            ) from exc

        self.np = np
        self.trt = trt
        self.contract = dict(manifest.isaac_contract or {})
        layout = self.contract.get("observation_layout")
        if layout != OBSERVATION_LAYOUT:
            raise RuntimeError(
                f"TensorRT runner does not support observation layout {layout!r}"
            )
        self.observation_state = T60ObservationState()
        self.cuda = None
        self.device_input = None
        self.device_output = None
        self.stream = None
        self._closed = False

        model_path = Path(manifest.model_path)
        if not model_path.is_file():
            raise RuntimeError(f"TensorRT model does not exist: {model_path}")

        self.logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(self.logger)
        explicit_batch = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        network = builder.create_network(explicit_batch)
        parser = trt.OnnxParser(network, self.logger)
        if not parser.parse(model_path.read_bytes()):
            errors = "; ".join(
                str(parser.get_error(index)) for index in range(parser.num_errors)
            )
            raise RuntimeError(f"TensorRT failed to parse {model_path}: {errors}")
        serialized_engine = builder.build_serialized_network(
            network, builder.create_builder_config()
        )
        if serialized_engine is None:
            raise RuntimeError(f"TensorRT failed to build {model_path}")

        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(serialized_engine)
        if self.engine is None:
            raise RuntimeError(f"TensorRT failed to deserialize {model_path}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("TensorRT failed to create an execution context")

        inputs = []
        outputs = []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                inputs.append(name)
            elif mode == trt.TensorIOMode.OUTPUT:
                outputs.append(name)
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError(
                f"TensorRT policy requires one input and one output, got "
                f"{inputs} and {outputs}"
            )

        self.input_name = inputs[0]
        self.output_name = outputs[0]
        expected_input_name = str(self.contract.get("input_name", "obs"))
        expected_output_name = str(self.contract.get("output_name", "actions"))
        if self.input_name != expected_input_name or self.output_name != expected_output_name:
            raise RuntimeError(
                f"TensorRT tensor names are {self.input_name!r}/{self.output_name!r}, "
                f"expected {expected_input_name!r}/{expected_output_name!r}"
            )

        input_shape = self._static_shape(self.input_name)
        output_shape = self._static_shape(self.output_name)
        if input_shape != (1, self.observation_state.observation_dim):
            raise RuntimeError(
                f"TensorRT input shape is {input_shape}, expected "
                f"(1, {self.observation_state.observation_dim})"
            )
        if output_shape != (1, self.observation_state.action_dim):
            raise RuntimeError(
                f"TensorRT output shape is {output_shape}, expected "
                f"(1, {self.observation_state.action_dim})"
            )
        if self.engine.get_tensor_dtype(self.input_name) != trt.float32:
            raise RuntimeError("TensorRT policy input must be float32")
        if self.engine.get_tensor_dtype(self.output_name) != trt.float32:
            raise RuntimeError("TensorRT policy output must be float32")

        self.host_input = np.empty(input_shape, dtype=np.float32)
        self.host_output = np.empty(output_shape, dtype=np.float32)
        try:
            self.cuda = _CudaRuntime()
            self.device_input = self.cuda.allocate(self.host_input.nbytes)
            self.device_output = self.cuda.allocate(self.host_output.nbytes)
            self.stream = self.cuda.create_stream()
            if not self.context.set_tensor_address(
                self.input_name, self.device_input.value
            ):
                raise RuntimeError("TensorRT rejected the input buffer address")
            if not self.context.set_tensor_address(
                self.output_name, self.device_output.value
            ):
                raise RuntimeError("TensorRT rejected the output buffer address")
        except Exception:
            self.close()
            raise

    def _static_shape(self, name):
        shape = tuple(int(value) for value in self.engine.get_tensor_shape(name))
        if not shape or any(value <= 0 for value in shape):
            raise RuntimeError(f"TensorRT tensor {name!r} has dynamic shape {shape}")
        return shape

    def run(self, observation):
        policy_input = self.observation_state.build(observation)
        self.np.copyto(self.host_input, policy_input)
        self.cuda.copy(
            self.device_input,
            ctypes.c_void_p(self.host_input.ctypes.data),
            self.host_input.nbytes,
            self.cuda.host_to_device,
        )
        if not self.context.execute_async_v3(self.stream.value):
            raise RuntimeError("TensorRT execute_async_v3 returned false")
        self.cuda.synchronize(self.stream)
        self.cuda.copy(
            ctypes.c_void_p(self.host_output.ctypes.data),
            self.device_output,
            self.host_output.nbytes,
            self.cuda.device_to_host,
        )
        action = self.host_output.reshape(-1).copy()
        self.observation_state.commit()
        return action.tolist()

    def reset(self):
        self.observation_state.reset()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.cuda is None:
            return
        try:
            self.cuda.destroy_stream(self.stream)
        finally:
            try:
                self.cuda.free(self.device_output)
            finally:
                self.cuda.free(self.device_input)
        self.stream = None
        self.device_output = None
        self.device_input = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
