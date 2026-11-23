"""Triple-buffered shared memory for runtime image delivery."""

from __future__ import annotations

import os
import struct
import uuid
from multiprocessing import resource_tracker, shared_memory

import msgspec


class ImageFrameRef(msgspec.Struct, kw_only=True, forbid_unknown_fields=True):
    schema_key: str
    shared_memory_name: str
    slot: int
    slot_size: int
    data_size: int
    sequence: int
    shape: list[int]
    format: str
    is_padded: bool = False


IMAGE_SLOT_COUNT = 3
IMAGE_SLOT_HEADER_SIZE = struct.calcsize("<Q")


class SharedImageWriter:
    """Own shared-memory buffers and publish immutable slot references."""

    def __init__(self) -> None:
        self.buffers: dict[str, tuple[shared_memory.SharedMemory, int]] = {}
        self.retired_buffers: list[shared_memory.SharedMemory] = []
        self.sequences: dict[str, int] = {}

    def write(
        self,
        *,
        stream_key: str,
        schema_key: str,
        data: bytes,
        shape: tuple[int, ...],
        format: str,
        is_padded: bool = False,
    ) -> ImageFrameRef:
        data = bytes(data)
        required_slot_size = max(len(data), 1)
        buffer_entry = self.buffers.get(stream_key)
        if buffer_entry is None or buffer_entry[1] < required_slot_size:
            if buffer_entry is not None:
                self.retired_buffers.append(buffer_entry[0])
            slot_size = 1 << (required_slot_size - 1).bit_length()
            name = f"robot_runtime_{os.getpid()}_{uuid.uuid4().hex}"
            buffer = shared_memory.SharedMemory(
                name=name,
                create=True,
                size=(IMAGE_SLOT_HEADER_SIZE + slot_size) * IMAGE_SLOT_COUNT,
            )
            self.buffers[stream_key] = (buffer, slot_size)
        else:
            buffer, slot_size = buffer_entry

        sequence = self.sequences.get(stream_key, 0) + 1
        self.sequences[stream_key] = sequence
        slot = (sequence - 1) % IMAGE_SLOT_COUNT
        slot_offset = slot * (IMAGE_SLOT_HEADER_SIZE + slot_size)
        data_offset = slot_offset + IMAGE_SLOT_HEADER_SIZE
        struct.pack_into("<Q", buffer.buf, slot_offset, 0)
        buffer.buf[data_offset : data_offset + len(data)] = data
        struct.pack_into("<Q", buffer.buf, slot_offset, sequence)
        return ImageFrameRef(
            schema_key=schema_key,
            shared_memory_name=buffer.name,
            slot=slot,
            slot_size=slot_size,
            data_size=len(data),
            sequence=sequence,
            shape=list(shape),
            format=format,
            is_padded=is_padded,
        )

    def close(self) -> None:
        buffers = [
            *(entry[0] for entry in self.buffers.values()),
            *self.retired_buffers,
        ]
        self.buffers.clear()
        self.retired_buffers.clear()
        self.sequences.clear()
        for buffer in buffers:
            try:
                buffer.close()
            finally:
                try:
                    buffer.unlink()
                except FileNotFoundError:
                    pass


class SharedImageReader:
    """Attach to runtime-owned image buffers and copy one referenced slot."""

    def __init__(self) -> None:
        self.buffers: dict[str, shared_memory.SharedMemory] = {}

    def read(self, frame_ref: ImageFrameRef) -> bytes | None:
        if (
            frame_ref.slot < 0
            or frame_ref.slot >= IMAGE_SLOT_COUNT
            or frame_ref.slot_size <= 0
            or frame_ref.data_size < 0
            or frame_ref.data_size > frame_ref.slot_size
        ):
            return None
        buffer = self.buffers.get(frame_ref.shared_memory_name)
        if buffer is None:
            buffer = shared_memory.SharedMemory(
                name=frame_ref.shared_memory_name,
                create=False,
            )
            # The runtime owns unlinking. Prevent this attaching process from
            # removing the live segment when its resource tracker exits.
            resource_tracker.unregister(buffer._name, "shared_memory")
            self.buffers[frame_ref.shared_memory_name] = buffer
        slot_offset = frame_ref.slot * (
            IMAGE_SLOT_HEADER_SIZE + frame_ref.slot_size
        )
        data_offset = slot_offset + IMAGE_SLOT_HEADER_SIZE
        sequence_before = struct.unpack_from(
            "<Q",
            buffer.buf,
            slot_offset,
        )[0]
        if sequence_before != frame_ref.sequence:
            return None
        data = bytes(
            buffer.buf[data_offset : data_offset + frame_ref.data_size]
        )
        sequence_after = struct.unpack_from(
            "<Q",
            buffer.buf,
            slot_offset,
        )[0]
        if sequence_after != frame_ref.sequence:
            return None
        return data

    def close(self) -> None:
        for buffer in self.buffers.values():
            buffer.close()
        self.buffers.clear()
