from __future__ import annotations

import numpy as np

from pybh._schema import Typed


class JPEGImage(Typed):

    def _jpeg_bytes(self) -> bytes:
        data = bytes(self._r._data)
        soi = data.find(b'\xff\xd8')
        return data[soi:] if soi >= 0 else data

    def to_numpy(self) -> np.ndarray:
        from turbojpeg import TurboJPEG, TJPF_CMYK
        cmyk = TurboJPEG().decode(self._jpeg_bytes(), pixel_format=TJPF_CMYK)
        rows, cols, _ = cmyk.shape
        a = cmyk.astype(np.float32)
        y0, u, y1, v = a[..., 0], a[..., 1], a[..., 2], a[..., 3]

        def yuv2rgb(y, cb, cr):
            cb = cb - 128.0
            cr = cr - 128.0
            return np.stack([y + 1.402 * cr,
                             y - 0.344136 * cb - 0.714136 * cr,
                             y + 1.772 * cb], axis=-1)

        rgb = np.empty((rows, cols * 2, 3), dtype=np.float32)
        rgb[:, 0::2] = yuv2rgb(y0, u, v)
        rgb[:, 1::2] = yuv2rgb(y1, u, v)
        return np.clip(rgb, 0, 255).astype(np.uint8)

    def to_rgb(self) -> np.ndarray:
        return self.to_numpy()

    def to_bgr(self) -> np.ndarray:
        return self.to_numpy()[..., ::-1]

SPECIAL: dict[str, type[Typed]] = {
    'JPEGImage': JPEGImage,
}
