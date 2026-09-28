"""HDF5 data writer for robot and camera data."""

import os
import h5py
import numpy as np

CHUNK_SIZE = 1000  # HDF5 pre-allocation chunk size


class RobotDataWriter:
    """Writes synchronized robot data (arms + hands) to HDF5."""

    def __init__(self, episode_path: str):
        self.episode_path = episode_path
        self.h5_path = os.path.join(episode_path, "robot_data.h5")
        self._file: h5py.File = None
        self._datasets = {}
        self._dataset_sizes = {}
        self._idx = 0

    def open(self):
        """Create HDF5 file and datasets."""
        os.makedirs(self.episode_path, exist_ok=True)
        self._file = h5py.File(self.h5_path, "w")

        # Define dataset specs: (name, dims, dtype)
        specs = [
            ("left_arm/joints", (7,), "f4"),
            ("left_arm/tcp_pose", (7,), "f4"),  # xyz + rpy
            ("right_arm/joints", (7,), "f4"),
            ("right_arm/tcp_pose", (7,), "f4"),
            ("left_hand/joints", (12,), "f4"),
            ("right_hand/joints", (12,), "f4"),
            ("timestamps", (), "f8"),
        ]

        for name, dims, dtype in specs:
            if dims:
                shape = (0,) + dims
                maxshape = (None,) + dims
            else:
                shape = (0,)
                maxshape = (None,)
            self._datasets[name] = self._file.create_dataset(
                name, shape=shape, maxshape=maxshape, dtype=dtype
            )
            self._dataset_sizes[name] = 0

        return self

    def write_frame(
        self,
        left_arm_joints,
        left_arm_tcp,
        right_arm_joints,
        right_arm_tcp,
        left_hand_joints,
        right_hand_joints,
        timestamp: float,
    ):
        """Write a single synchronized data frame."""
        if self._file is None:
            return

        # Only require left_arm_joints to be present (most critical source)
        if left_arm_joints is None and right_arm_joints is None:
            return  # No arm data at all, skip

        idx = self._idx

        # Resize datasets if needed
        for name in self._datasets:
            if idx >= self._dataset_sizes[name]:
                self._dataset_sizes[name] += CHUNK_SIZE
                if self._datasets[name].ndim == 1:
                    self._datasets[name].resize((self._dataset_sizes[name],))
                else:
                    new_shape = (self._dataset_sizes[name],) + self._datasets[name].shape[1:]
                    self._datasets[name].resize(new_shape)

        # Write data
        self._datasets["left_arm/joints"][idx] = np.asarray(left_arm_joints, dtype=np.float32)
        self._datasets["left_arm/tcp_pose"][idx] = np.asarray(left_arm_tcp, dtype=np.float32)
        self._datasets["right_arm/joints"][idx] = np.asarray(right_arm_joints, dtype=np.float32)
        self._datasets["right_arm/tcp_pose"][idx] = np.asarray(right_arm_tcp, dtype=np.float32)
        self._datasets["left_hand/joints"][idx] = np.asarray(left_hand_joints, dtype=np.float32)
        self._datasets["right_hand/joints"][idx] = np.asarray(right_hand_joints, dtype=np.float32)
        self._datasets["timestamps"][idx] = timestamp

        self._idx += 1

    @property
    def frame_count(self) -> int:
        return self._idx

    def close(self):
        """Resize to actual frame count and close."""
        if self._file is None:
            return
        final_size = self._idx
        for name, ds in self._datasets.items():
            if ds.ndim == 1:
                ds.resize((final_size,))
            else:
                ds.resize((final_size,) + ds.shape[1:])
        self._file.close()
        self._file = None
        print(f"[RobotWriter] Saved {final_size} frames -> {self.h5_path}")


class CameraDataWriter:
    """Writes camera image data to HDF5 (one file per camera set)."""

    def __init__(self, episode_path: str, cam_ids: list):
        self.episode_path = episode_path
        self.cam_ids = cam_ids
        self.h5_path = os.path.join(episode_path, "camera_data.h5")
        self._file: h5py.File = None
        self._image_dsets = {}
        self._ts_dsets = {}
        self._dset_sizes = {}
        self._idxs = {}

    def open(self):
        """Create HDF5 file and per-camera datasets."""
        os.makedirs(self.episode_path, exist_ok=True)
        self._file = h5py.File(self.h5_path, "w")

        for cam_id in self.cam_ids:
            self._image_dsets[cam_id] = self._file.create_dataset(
                f"{cam_id}/images",
                shape=(0,),
                maxshape=(None,),
                dtype=h5py.special_dtype(vlen=np.dtype("uint8")),
            )
            self._ts_dsets[cam_id] = self._file.create_dataset(
                f"{cam_id}/timestamps",
                shape=(0,),
                maxshape=(None,),
                dtype="f8",
            )
            self._dset_sizes[cam_id] = 0
            self._idxs[cam_id] = 0

        return self

    def write_image(self, cam_id: str, jpeg_bytes: bytes, timestamp: float):
        """Write a single camera image with timestamp."""
        if self._file is None:
            return

        i = self._idxs[cam_id]

        if i >= self._dset_sizes[cam_id]:
            self._dset_sizes[cam_id] += CHUNK_SIZE
            self._image_dsets[cam_id].resize((self._dset_sizes[cam_id],))
            self._ts_dsets[cam_id].resize((self._dset_sizes[cam_id],))

        self._image_dsets[cam_id][i] = np.frombuffer(jpeg_bytes, dtype="uint8")
        self._ts_dsets[cam_id][i] = timestamp
        self._idxs[cam_id] += 1

    def close(self):
        """Resize to actual counts and close."""
        if self._file is None:
            return
        for cam_id in self.cam_ids:
            final_size = self._idxs[cam_id]
            self._image_dsets[cam_id].resize((final_size,))
            self._ts_dsets[cam_id].resize((final_size,))
        self._file.close()
        self._file = None
        counts = {cid: self._idxs[cid] for cid in self.cam_ids}
        print(f"[CameraWriter] Saved {counts} -> {self.h5_path}")
