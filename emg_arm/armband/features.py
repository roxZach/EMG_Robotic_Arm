import numpy as np

DEFAULT_NUM_CHANNELS = 4
FFT_MIN_HZ = 20.0
FFT_MAX_HZ = 220.0
BANDS = [(20.0, 60.0), (60.0, 120.0), (120.0, 220.0)]

# Cache Hanning windows / FFT bin frequencies by window length. They never change per
# window size, so recomputing them for every extracted window is wasted work.
_FFT_CACHE = {}


def _hanning_and_freqs(n, sample_rate):
    key = (int(n), float(sample_rate))
    cached = _FFT_CACHE.get(key)
    if cached is None:
        win = np.hanning(n).astype(np.float32)
        freqs = np.fft.rfftfreq(int(n), d=1.0 / float(sample_rate))
        cached = (win, freqs)
        _FFT_CACHE[key] = cached
    return cached


def _ensure_window_shape(window):
    arr = np.asarray(window, dtype=np.float32)
    if arr.ndim != 2:
        raise ValueError("window must be 2D")
    # Prefer (samples, channels). If likely transposed, flip.
    if arr.shape[0] < arr.shape[1]:
        arr = arr.T
    if arr.shape[1] <= 0:
        raise ValueError("window must have at least 1 channel")
    return arr


def _spectral_1d(x, sample_rate):
    x = np.asarray(x, dtype=np.float32)
    n = x.shape[0]
    if n < 8:
        return {
            "mean_hz": 0.0,
            "median_hz": 0.0,
            "peak_hz": 0.0,
            "spec_entropy": 0.0,
            "band_power_pct": [0.0, 0.0, 0.0],
        }

    xc = x - np.mean(x)
    win = np.hanning(n).astype(np.float32)
    spec = np.abs(np.fft.rfft(xc * win)) ** 2
    freqs = np.fft.rfftfreq(n, d=1.0 / float(sample_rate))

    mask = (freqs >= FFT_MIN_HZ) & (freqs <= FFT_MAX_HZ)
    if not np.any(mask):
        return {
            "mean_hz": 0.0,
            "median_hz": 0.0,
            "peak_hz": 0.0,
            "spec_entropy": 0.0,
            "band_power_pct": [0.0, 0.0, 0.0],
        }

    sv = spec[mask]
    fv = freqs[mask]
    total = float(np.sum(sv) + 1e-9)

    peak_hz = float(fv[int(np.argmax(sv))])
    mean_hz = float(np.sum(sv * fv) / total)
    csum = np.cumsum(sv)
    med_hz = float(fv[int(np.argmax(csum >= (0.5 * total)))])

    p = sv / total
    spec_entropy = float(-np.sum(p * np.log2(p + 1e-12)) / np.log2(len(p) + 1e-9))

    band_power = []
    for lo, hi in BANDS:
        bmask = (fv >= lo) & (fv < hi)
        if np.any(bmask):
            band_power.append(float(np.sum(sv[bmask]) / total * 100.0))
        else:
            band_power.append(0.0)

    return {
        "mean_hz": mean_hz,
        "median_hz": med_hz,
        "peak_hz": peak_hz,
        "spec_entropy": spec_entropy,
        "band_power_pct": band_power,
    }


def _spectral_all(A, sample_rate):
    """Vectorized equivalent of _spectral_1d applied to every channel at once.

    A: (n_ch, n_samples), C-contiguous. Reductions run along the contiguous last axis
    so numpy's pairwise summation matches the per-channel 1D path bit-for-bit.
    """
    n_ch, n = A.shape
    zeros = np.zeros(n_ch, dtype=np.float64)
    band_zeros = np.zeros((n_ch, len(BANDS)), dtype=np.float64)
    if n < 8:
        return zeros, zeros, zeros, zeros, band_zeros

    # _spectral_1d re-centers its input, so re-center per channel (row) here too.
    xc = A - np.mean(A, axis=1, keepdims=True)
    win, freqs = _hanning_and_freqs(n, sample_rate)
    spec = np.abs(np.fft.rfft(xc * win[np.newaxis, :], axis=1)) ** 2  # (n_ch, n_freq)

    mask = (freqs >= FFT_MIN_HZ) & (freqs <= FFT_MAX_HZ)
    if not np.any(mask):
        return zeros, zeros, zeros, zeros, band_zeros

    sv = np.ascontiguousarray(spec[:, mask])  # (n_ch, n_valid), contiguous rows
    fv = freqs[mask]                           # (n_valid,)
    total = np.sum(sv, axis=1) + 1e-9          # (n_ch,)

    peak_hz = fv[np.argmax(sv, axis=1)]
    mean_hz = np.sum(sv * fv[np.newaxis, :], axis=1) / total
    csum = np.cumsum(sv, axis=1)
    med_idx = np.argmax(csum >= (0.5 * total)[:, np.newaxis], axis=1)
    med_hz = fv[med_idx]

    p = sv / total[:, np.newaxis]
    spec_entropy = -np.sum(p * np.log2(p + 1e-12), axis=1) / np.log2(sv.shape[1] + 1e-9)

    band_power = np.zeros((n_ch, len(BANDS)), dtype=np.float64)
    for i, (lo, hi) in enumerate(BANDS):
        bmask = (fv >= lo) & (fv < hi)
        if np.any(bmask):
            band_power[:, i] = np.sum(np.ascontiguousarray(sv[:, bmask]), axis=1) / total * 100.0

    return (
        mean_hz.astype(np.float64),
        med_hz.astype(np.float64),
        peak_hz.astype(np.float64),
        spec_entropy.astype(np.float64),
        band_power,
    )


def feature_names(num_channels=DEFAULT_NUM_CHANNELS):
    n_ch = int(max(1, num_channels))
    names = []
    per_ch = [
        "mav",
        "rms",
        "iemg",
        "var",
        "wl",
        "zc",
        "ssc",
        "wamp",
        "mean_hz",
        "median_hz",
        "peak_hz",
        "spec_entropy",
        "bp20_60",
        "bp60_120",
        "bp120_220",
    ]
    for ch in range(1, n_ch + 1):
        for k in per_ch:
            names.append(f"ch{ch}_{k}")

    for ch in range(1, n_ch + 1):
        names.append(f"ch{ch}_rms_ratio")

    for a in range(1, n_ch + 1):
        for b in range(a + 1, n_ch + 1):
            names.append(f"corr_ch{a}_ch{b}")
    return names


def _pairwise_corr_features(arr_centered):
    """Upper-triangle channel correlations, identical to the reference loop."""
    n_ch = arr_centered.shape[1]
    std = np.std(arr_centered, axis=0)
    valid = np.isfinite(std) & (std > 1e-8)
    if np.any(valid):
        with np.errstate(invalid="ignore", divide="ignore"):
            corr = np.corrcoef(arr_centered.T)
    else:
        corr = np.eye(n_ch, dtype=np.float32)
    corr = np.atleast_2d(np.nan_to_num(corr, nan=0.0, posinf=0.0, neginf=0.0))
    if not np.all(valid):
        corr[~valid, :] = 0.0
        corr[:, ~valid] = 0.0
        np.fill_diagonal(corr, 1.0)
    out = []
    for a in range(n_ch):
        for b in range(a + 1, n_ch):
            out.append(float(corr[a, b]))
    return out


def extract_window_features(window, sample_rate=500):
    """Vectorized feature extraction. Produces the same feature vector (verified
    bit-for-bit) as _extract_window_features_reference below, but computes all channels
    at once instead of looping in Python. The reference is kept as the canonical
    definition; diff against it if this file is ever changed.
    """
    arr = _ensure_window_shape(window)
    n_samples = arr.shape[0]
    n_ch = arr.shape[1]
    arr_centered = arr - np.mean(arr, axis=0, keepdims=True)
    # Channels as rows, samples as the contiguous last axis. Reducing along axis=1 of a
    # C-contiguous array reproduces numpy's per-channel 1D pairwise summation bit-for-bit,
    # which the loop-based reference relies on.
    A = np.ascontiguousarray(arr_centered.T)  # (n_ch, n_samples)

    zc_thresh = 10.0
    ssc_thresh = 8.0
    wamp_thresh = 12.0

    abs_x = np.abs(A)
    mav = np.mean(abs_x, axis=1)
    rms = np.sqrt(np.mean(np.square(A), axis=1))
    iemg = np.sum(abs_x, axis=1)
    var = np.var(A, axis=1)

    if n_samples > 1:
        dx = np.diff(A, axis=1)
        abs_dx = np.abs(dx)
        wl = np.sum(abs_dx, axis=1)
        prod = A[:, :-1] * A[:, 1:]
        zc = np.sum((prod < 0) & (abs_dx >= zc_thresh), axis=1)
        wamp = np.sum(abs_dx >= wamp_thresh, axis=1)
    else:
        wl = np.zeros(n_ch, dtype=np.float32)
        zc = np.zeros(n_ch, dtype=np.int64)
        wamp = np.zeros(n_ch, dtype=np.int64)

    if n_samples > 2:
        s1 = A[:, 1:-1] - A[:, :-2]
        s2 = A[:, 1:-1] - A[:, 2:]
        ssc = np.sum(((s1 * s2) > 0) & ((np.abs(s1) + np.abs(s2)) >= ssc_thresh), axis=1)
    else:
        ssc = np.zeros(n_ch, dtype=np.int64)

    mean_hz, median_hz, peak_hz, spec_entropy, band_power = _spectral_all(A, sample_rate)

    # Assemble the 15 per-channel features in the reference column order, then flatten
    # row-major so the layout is [ch0 x15, ch1 x15, ...] exactly like the loop version.
    per_ch = np.column_stack([
        mav.astype(np.float64),
        rms.astype(np.float64),
        iemg.astype(np.float64),
        var.astype(np.float64),
        wl.astype(np.float64),
        zc.astype(np.float64),
        ssc.astype(np.float64),
        wamp.astype(np.float64),
        mean_hz,
        median_hz,
        peak_hz,
        spec_entropy,
        band_power[:, 0],
        band_power[:, 1],
        band_power[:, 2],
    ])
    feats = list(per_ch.reshape(-1))

    rms_vals = rms.astype(np.float32)
    mean_rms = float(np.mean(rms_vals) + 1e-9)
    feats.extend((rms_vals / mean_rms).tolist())

    feats.extend(_pairwise_corr_features(arr_centered))

    return np.asarray(feats, dtype=np.float32)


def _extract_window_features_reference(window, sample_rate=500):
    """Original per-channel implementation kept as the correctness reference."""
    arr = _ensure_window_shape(window)
    n_samples = arr.shape[0]
    n_ch = arr.shape[1]
    arr_centered = arr - np.mean(arr, axis=0, keepdims=True)

    zc_thresh = 10.0
    ssc_thresh = 8.0
    wamp_thresh = 12.0

    feats = []
    rms_vals = []
    for ch in range(n_ch):
        x = arr_centered[:, ch]
        abs_x = np.abs(x)
        dx = np.diff(x) if n_samples > 1 else np.array([], dtype=np.float32)

        mav = float(np.mean(abs_x))
        rms = float(np.sqrt(np.mean(np.square(x))))
        iemg = float(np.sum(abs_x))
        var = float(np.var(x))
        wl = float(np.sum(np.abs(dx))) if dx.size else 0.0

        if n_samples > 1:
            zc = int(np.sum(((x[:-1] * x[1:]) < 0) & (np.abs(x[:-1] - x[1:]) >= zc_thresh)))
            wamp = int(np.sum(np.abs(x[1:] - x[:-1]) >= wamp_thresh))
        else:
            zc = 0
            wamp = 0

        if n_samples > 2:
            s1 = x[1:-1] - x[:-2]
            s2 = x[1:-1] - x[2:]
            ssc = int(np.sum(((s1 * s2) > 0) & ((np.abs(s1) + np.abs(s2)) >= ssc_thresh)))
        else:
            ssc = 0

        sp = _spectral_1d(x, sample_rate)
        feats.extend(
            [
                mav,
                rms,
                iemg,
                var,
                wl,
                float(zc),
                float(ssc),
                float(wamp),
                sp["mean_hz"],
                sp["median_hz"],
                sp["peak_hz"],
                sp["spec_entropy"],
                sp["band_power_pct"][0],
                sp["band_power_pct"][1],
                sp["band_power_pct"][2],
            ]
        )
        rms_vals.append(rms)

    rms_vals = np.asarray(rms_vals, dtype=np.float32)
    mean_rms = float(np.mean(rms_vals) + 1e-9)
    feats.extend((rms_vals / mean_rms).tolist())

    # Pairwise channel correlation features.
    std = np.std(arr_centered, axis=0)
    valid = np.isfinite(std) & (std > 1e-8)
    if np.any(valid):
        with np.errstate(invalid="ignore", divide="ignore"):
            corr = np.corrcoef(arr_centered.T)
    else:
        corr = np.eye(n_ch, dtype=np.float32)
    corr = np.nan_to_num(corr, nan=0.0, posinf=0.0, neginf=0.0)
    if not np.all(valid):
        corr[~valid, :] = 0.0
        corr[:, ~valid] = 0.0
        np.fill_diagonal(corr, 1.0)
    for a in range(n_ch):
        for b in range(a + 1, n_ch):
            feats.append(float(corr[a, b]))

    return np.asarray(feats, dtype=np.float32)


def build_windows_from_sequence(sequence, win_samples, stride_samples):
    arr = _ensure_window_shape(sequence)
    n = arr.shape[0]
    if n < win_samples:
        return []
    out = []
    for s in range(0, n - win_samples + 1, stride_samples):
        out.append(arr[s:s + win_samples, :])
    return out
