from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep
from typing import Any, Optional, Union

import cv2
import numpy as np


SourceValue = Union[int, str, Path]


@dataclass(frozen=True)
class FramePacket:
    """A frame plus metadata required by downstream vision modules."""

    image: np.ndarray
    timestamp: float
    frame_id: int
    source_name: str
    reconnect_count: int = 0


@dataclass
class CameraConfig:
    """Settings whose supported ranges depend on the actual camera/driver."""

    width: int = 1280
    height: int = 720
    fps: float = 30.0
    backend: str = "auto"
    fourcc: str = "MJPG"
    buffer_size: int = 1
    warmup_frames: int = 10
    open_retries: int = 3
    max_read_failures: int = 5
    reconnect_retries: int = 5
    reconnect_delay: float = 0.5
    exposure: Optional[float] = None
    auto_exposure: Optional[float] = None
    gain: Optional[float] = None
    brightness: Optional[float] = None
    contrast: Optional[float] = None
    saturation: Optional[float] = None
    sharpness: Optional[float] = None
    white_balance: Optional[float] = None
    auto_white_balance: Optional[float] = None
    focus: Optional[float] = None
    auto_focus: Optional[float] = None
    # Picamera2/libcamera controls for Raspberry Pi CSI cameras. ExposureTime
    # uses microseconds; colour_gains is (red_gain, blue_gain); lens_position
    # uses dioptres as defined by libcamera.
    exposure_time_us: Optional[int] = None
    analogue_gain: Optional[float] = None
    colour_gains: Optional[tuple[float, float]] = None
    lens_position: Optional[float] = None
    csi_auto_exposure: bool = True
    csi_auto_white_balance: bool = True
    csi_auto_focus: bool = True


class ImageSource:
    """Robust reader for an image, video, USB/UVC camera, or Pi CSI camera.

    Camera reads recover from short failures and attempt to reopen a disconnected
    device. The public read() method is intentionally independent of OpenCV's
    backend so detection code can use recorded video and a real robot camera in
    exactly the same way.
    """

    IMAGE_EXTENSIONS = {".bmp", ".jpg", ".jpeg", ".png", ".tif", ".tiff", ".webp"}
    BACKENDS = {
        "auto": cv2.CAP_ANY,
        "any": cv2.CAP_ANY,
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "v4l2": cv2.CAP_V4L2,
        "picamera2": None,
    }
    CONTROL_PROPERTIES = {
        "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
        "exposure": cv2.CAP_PROP_EXPOSURE,
        "gain": cv2.CAP_PROP_GAIN,
        "brightness": cv2.CAP_PROP_BRIGHTNESS,
        "contrast": cv2.CAP_PROP_CONTRAST,
        "saturation": cv2.CAP_PROP_SATURATION,
        "sharpness": cv2.CAP_PROP_SHARPNESS,
        "auto_white_balance": cv2.CAP_PROP_AUTO_WB,
        "white_balance": cv2.CAP_PROP_WB_TEMPERATURE,
        "auto_focus": cv2.CAP_PROP_AUTOFOCUS,
        "focus": cv2.CAP_PROP_FOCUS,
    }

    def __init__(
        self,
        source: SourceValue,
        camera_config: Optional[CameraConfig] = None,
        loop_video: bool = False,
    ) -> None:
        self.source = source
        self.camera_config = camera_config or CameraConfig()
        self.loop_video = loop_video
        self.frame_id = 0
        self.reconnect_count = 0
        self._consecutive_failures = 0
        self._capture: Optional[cv2.VideoCapture] = None
        self._picamera: Optional[Any] = None
        self._still_image: Optional[np.ndarray] = None
        self._is_camera = isinstance(source, int)
        self._source_name = f"camera:{source}" if self._is_camera else str(source)
        self._open()

    def _open(self) -> None:
        if self._is_camera:
            self._open_camera()
            return

        path = Path(self.source)
        if not path.exists():
            raise FileNotFoundError(f"Input file does not exist: {path.resolve()}")
        if path.suffix.lower() in self.IMAGE_EXTENSIONS:
            self._still_image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if self._still_image is None:
                raise RuntimeError(f"OpenCV could not decode image: {path}")
            return
        self._capture = cv2.VideoCapture(str(path))
        self._ensure_opened()

    def _open_camera(self) -> None:
        config = self.camera_config
        backend_name = config.backend.lower()
        if backend_name not in self.BACKENDS:
            raise ValueError(f"Unknown backend '{config.backend}'. Choose: {', '.join(self.BACKENDS)}")
        if backend_name == "picamera2":
            self._open_picamera2()
            return
        backend = self.BACKENDS[backend_name]

        self.release()
        last_error = "camera did not open"
        for attempt in range(1, max(config.open_retries, 1) + 1):
            capture = cv2.VideoCapture(int(self.source), backend)
            if capture.isOpened():
                self._capture = capture
                self._configure_camera()
                self._discard_warmup_frames()
                self._consecutive_failures = 0
                return
            capture.release()
            last_error = f"attempt {attempt}/{config.open_retries} failed"
            sleep(config.reconnect_delay)
        raise RuntimeError(
            f"Could not open {self._source_name} with backend={backend_name}: {last_error}. "
            "Close other camera applications and try another index/backend."
        )

    @staticmethod
    def _picamera2_class():
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 is required for backend=picamera2. On Raspberry Pi OS run "
                "'sudo apt install python3-picamera2', then use a venv created with "
                "'python3 -m venv --system-site-packages .venv'."
            ) from exc
        return Picamera2

    def _open_picamera2(self) -> None:
        config = self.camera_config
        self.release()
        Picamera2 = self._picamera2_class()
        last_error = "camera did not open"
        for attempt in range(1, max(config.open_retries, 1) + 1):
            camera = None
            try:
                camera = Picamera2(camera_num=int(self.source))
                controls: dict[str, object] = {"FrameRate": float(config.fps)}
                if config.exposure_time_us is not None or config.analogue_gain is not None:
                    controls["AeEnable"] = False
                else:
                    controls["AeEnable"] = bool(config.csi_auto_exposure)
                if config.exposure_time_us is not None:
                    controls["ExposureTime"] = int(config.exposure_time_us)
                if config.analogue_gain is not None:
                    controls["AnalogueGain"] = float(config.analogue_gain)
                controls["AwbEnable"] = bool(config.csi_auto_white_balance)
                if config.colour_gains is not None:
                    controls["AwbEnable"] = False
                    controls["ColourGains"] = tuple(map(float, config.colour_gains))
                available_controls = getattr(camera, "camera_controls", {})
                if config.lens_position is not None and "LensPosition" not in available_controls:
                    raise RuntimeError("this CSI camera does not support lens-position control")
                if config.lens_position is not None:
                    controls["AfMode"] = 0  # libcamera AfModeManual
                    controls["LensPosition"] = float(config.lens_position)
                elif config.csi_auto_focus and "AfMode" in available_controls:
                    controls["AfMode"] = 2  # libcamera AfModeContinuous

                video_config = camera.create_video_configuration(
                    main={"size": (int(config.width), int(config.height)), "format": "RGB888"},
                    controls=controls,
                    buffer_count=max(int(config.buffer_size) + 1, 2),
                    queue=False,
                )
                camera.configure(video_config)
                camera.start()
                self._picamera = camera
                self._discard_picamera_warmup_frames()
                self._consecutive_failures = 0
                return
            except Exception as exc:
                last_error = str(exc)
                if camera is not None:
                    try:
                        camera.close()
                    except Exception:
                        pass
                if attempt < max(config.open_retries, 1):
                    sleep(config.reconnect_delay)
        raise RuntimeError(
            f"Could not open {self._source_name} with backend=picamera2: {last_error}. "
            "Check 'rpicam-hello --list-cameras' and make sure no other process owns the camera."
        )

    def _discard_picamera_warmup_frames(self) -> None:
        assert self._picamera is not None
        for _ in range(max(self.camera_config.warmup_frames, 0)):
            self._picamera.capture_array("main")

    def _configure_camera(self) -> None:
        assert self._capture is not None
        config = self.camera_config

        # Set stream format before dimensions; many UVC drivers choose a mode
        # from the combination of FOURCC, resolution, and FPS.
        fourcc = config.fourcc.upper()
        if len(fourcc) != 4:
            raise ValueError("FOURCC must contain exactly four characters, e.g. MJPG or YUYV")
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
        self._capture.set(cv2.CAP_PROP_FPS, config.fps)
        if config.buffer_size >= 0:
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, config.buffer_size)

        # Set automatic modes before manual values. Driver-specific controls may
        # reject a value; actual values are always reported through properties().
        for name in ("auto_exposure", "auto_white_balance", "auto_focus"):
            self._set_control(name, getattr(config, name))
        for name in (
            "exposure", "gain", "brightness", "contrast", "saturation",
            "sharpness", "white_balance", "focus",
        ):
            self._set_control(name, getattr(config, name))

    def _set_control(self, name: str, value: Optional[float]) -> None:
        if value is None or self._capture is None:
            return
        accepted = self._capture.set(self.CONTROL_PROPERTIES[name], float(value))
        if not accepted:
            print(f"WARNING: camera driver rejected {name}={value}")

    def _discard_warmup_frames(self) -> None:
        assert self._capture is not None
        for _ in range(max(self.camera_config.warmup_frames, 0)):
            self._capture.read()

    def _ensure_opened(self) -> None:
        if self._capture is None or not self._capture.isOpened():
            raise RuntimeError(f"Could not open {self._source_name}")

    def _reconnect(self) -> bool:
        config = self.camera_config
        for attempt in range(1, max(config.reconnect_retries, 0) + 1):
            print(f"WARNING: reconnecting {self._source_name} ({attempt}/{config.reconnect_retries})")
            sleep(config.reconnect_delay)
            try:
                self._open_camera()
                self.reconnect_count += 1
                print(f"Camera reconnected: {self._source_name}")
                return True
            except RuntimeError as exc:
                print(f"WARNING: {exc}")
        return False

    @property
    def source_name(self) -> str:
        return self._source_name

    @property
    def is_still_image(self) -> bool:
        return self._still_image is not None

    @property
    def is_camera(self) -> bool:
        return self._is_camera

    def properties(self) -> dict[str, object]:
        if self._still_image is not None:
            height, width = self._still_image.shape[:2]
            return {"width": width, "height": height, "fps": 0.0, "backend": "image"}
        if self._picamera is not None:
            metadata = self._picamera.capture_metadata()
            return {
                "width": self.camera_config.width,
                "height": self.camera_config.height,
                "fps": self.camera_config.fps,
                "format": "RGB888 (OpenCV BGR array)",
                "backend": "picamera2",
                "camera_num": int(self.source),
                "exposure_time_us": metadata.get("ExposureTime"),
                "analogue_gain": metadata.get("AnalogueGain"),
                "colour_gains": metadata.get("ColourGains"),
                "lens_position": metadata.get("LensPosition"),
            }
        if self._capture is None:
            return {"width": 0, "height": 0, "fps": 0.0, "backend": "closed"}
        raw_fourcc = int(self._capture.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((raw_fourcc >> (8 * i)) & 0xFF) for i in range(4))
        result: dict[str, object] = {
            "width": int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": self._capture.get(cv2.CAP_PROP_FPS),
            "fourcc": fourcc,
            "backend": self._capture.getBackendName(),
            "buffer_size": self._capture.get(cv2.CAP_PROP_BUFFERSIZE),
        }
        for name, prop in self.CONTROL_PROPERTIES.items():
            result[name] = self._capture.get(prop)
        return result

    def read(self) -> Optional[FramePacket]:
        if self._still_image is not None:
            image = self._still_image.copy()
        elif self._picamera is not None:
            try:
                image = self._picamera.capture_array("main")
            except Exception as exc:
                print(f"WARNING: CSI camera read failed: {exc}")
                image = None
            if image is None or image.size == 0:
                self._consecutive_failures += 1
                if self._consecutive_failures < self.camera_config.max_read_failures:
                    return self.read()
                if not self._reconnect():
                    return None
                return self.read()
            self._consecutive_failures = 0
        else:
            assert self._capture is not None
            ok, image = self._capture.read()
            if not ok and self.loop_video and not self._is_camera:
                self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, image = self._capture.read()

            if not ok or image is None or image.size == 0:
                if not self._is_camera:
                    return None
                self._consecutive_failures += 1
                if self._consecutive_failures < self.camera_config.max_read_failures:
                    return self.read()
                if not self._reconnect():
                    return None
                assert self._capture is not None
                ok, image = self._capture.read()
                if not ok or image is None or image.size == 0:
                    return None
            self._consecutive_failures = 0

        packet = FramePacket(
            image=image,
            timestamp=monotonic(),
            frame_id=self.frame_id,
            source_name=self._source_name,
            reconnect_count=self.reconnect_count,
        )
        self.frame_id += 1
        return packet

    def release(self) -> None:
        if self._picamera is not None:
            try:
                self._picamera.stop()
            finally:
                self._picamera.close()
                self._picamera = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> "ImageSource":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.release()
