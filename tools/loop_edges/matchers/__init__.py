# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Matcher backends for export_loop_edges.py. Each backend exposes
#   match(frame_i, frame_j, max_matches) -> (xy_i, xy_j) | None
# where xy_i / xy_j are Nx2 float arrays of corresponding pixel coordinates in
# the two frames' images. Backprojection to metric 3D + robust pose live in the
# caller; a backend only produces 2D correspondences.
#
# LICENCE BOUNDARY — read before adding a backend:
#   * orb / xfeat / lightglue are Apache-2.0 / BSD (commercial-safe, shippable).
#   * mast3r / mapanything(-NC) are CC-BY-NC research oracles. They are gated
#     behind --allow-noncommercial and print a banner. Their edge files must be
#     treated as evaluation artefacts, never bundled into a commercial product.
#     Nothing here is compiled into the C++ binary; this whole tool runs
#     out-of-process and hands the C++ a plain JSON of relative-pose constraints.

import sys


def load_matcher(args):
    name = args.matcher
    if name == "orb":
        return OrbMatcher(nfeatures=3000, ratio=0.85)
    if name == "xfeat":
        return XFeatMatcher(device=args.device, weights=args.weights)
    if name == "lightglue":
        return LightGlueMatcher(device=args.device, extractor=args.variant or "aliked")
    if name in ("mast3r", "mapanything"):
        if not args.allow_noncommercial and _is_noncommercial(name, args.variant):
            sys.exit(
                f"[licence] '{name}' (variant '{args.variant}') is a "
                "non-commercial research model. Re-run with --allow-noncommercial "
                "to use it as an offline accuracy-ceiling ORACLE only; its output "
                "must not enter a commercial deliverable. See tools/loop_edges/"
                "README.md and docs/research/loop-closure-learned-matchers.md."
            )
        if name == "mast3r":
            return Mast3rMatcher(device=args.device, weights=args.weights)
        return MapAnythingMatcher(
            device=args.device, weights=args.weights, variant=args.variant or "apache"
        )
    raise ValueError(name)


def _is_noncommercial(name, variant):
    # MapAnything ships an Apache-2.0 checkpoint; only the 13-dataset one is NC.
    if name == "mapanything" and (variant or "apache") == "apache":
        return False
    return True


# --------------------------------------------------------------------------- #
# ORB (OpenCV, BSD) — zero extra deps, commercial-safe baseline / parity check #
# --------------------------------------------------------------------------- #
class OrbMatcher:
    def __init__(self, nfeatures=3000, ratio=0.85):
        import cv2

        self.cv2 = cv2
        self.orb = cv2.ORB_create(nfeatures=nfeatures)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.ratio = ratio
        self._cache = {}  # node_id -> (keypoints, descriptors)

    def _feat(self, frame):
        c = self._cache.get(frame.node_id)
        if c is None:
            c = self.orb.detectAndCompute(frame.gray, None)
            self._cache[frame.node_id] = c
        return c

    def match(self, fi, fj, max_matches=4000):
        cv2 = self.cv2
        ki, di = self._feat(fi)
        kj, dj = self._feat(fj)
        if di is None or dj is None or len(ki) < 4 or len(kj) < 4:
            return None
        knn = self.bf.knnMatch(di, dj, k=2)
        good = [
            m
            for pair in knn
            if len(pair) == 2
            for m in [pair[0]]
            if m.distance < self.ratio * pair[1].distance
        ]
        if len(good) < 3:
            return None
        good.sort(key=lambda m: m.distance)
        good = good[:max_matches]
        import numpy as np

        xy_i = np.array([ki[m.queryIdx].pt for m in good], np.float64)
        xy_j = np.array([kj[m.trainIdx].pt for m in good], np.float64)
        return xy_i, xy_j


# --------------------------------------------------------------------------- #
# XFeat (Apache-2.0) — commercial-safe learned features                        #
# --------------------------------------------------------------------------- #
class XFeatMatcher:
    def __init__(self, device="cuda", weights=None):
        import torch

        self.torch = torch
        self.device = device
        # Weights (xfeat.pt, ~6 MB, Apache-2.0) download from the
        # verlab/accelerated_features repo on first use and cache under
        # ~/.cache/torch/hub. trust_repo=True avoids the interactive prompt in
        # non-TTY contexts (subprocess / CI).
        self.model = torch.hub.load(
            "verlab/accelerated_features",
            "XFeat",
            pretrained=True,
            top_k=4096,
            trust_repo=True,
        )
        self.model = self.model.to(device).eval()

    def match(self, fi, fj, max_matches=4000):
        import numpy as np

        # match_xfeat accepts HxWx3 uint8 numpy directly and returns two Nx2
        # float arrays of pixel coords in the ORIGINAL image resolution; its
        # parse_input() handles /255 + BCHW internally. Pass colour (BGR) — XFeat
        # is colour-channel-agnostic and this avoids a needless channel expand.
        mkpts_0, mkpts_1 = self.model.match_xfeat(fi.color, fj.color)
        if mkpts_0 is None or len(mkpts_0) < 3:
            return None
        xy_i = np.asarray(mkpts_0, np.float64)[:max_matches]
        xy_j = np.asarray(mkpts_1, np.float64)[:max_matches]
        return xy_i, xy_j


# --------------------------------------------------------------------------- #
# LightGlue + ALIKED/DISK (Apache-2.0) — commercial-safe (NOT SuperPoint)      #
# --------------------------------------------------------------------------- #
class LightGlueMatcher:
    def __init__(self, device="cuda", extractor="aliked"):
        import torch
        from lightglue import ALIKED, DISK, LightGlue

        self.torch = torch
        self.device = device
        Ext = {"aliked": ALIKED, "disk": DISK}[extractor]
        self.ext = Ext(max_num_keypoints=2048).eval().to(device)
        self.lg = LightGlue(features=extractor).eval().to(device)

    def match(self, fi, fj, max_matches=4000):
        import numpy as np
        import torch
        from lightglue.utils import rbd

        def prep(gray):
            return torch.from_numpy(gray).float()[None, None].to(self.device) / 255.0

        with torch.inference_mode():
            f0 = self.ext.extract(prep(fi.gray))
            f1 = self.ext.extract(prep(fj.gray))
            out = self.lg({"image0": f0, "image1": f1})
            f0, f1, out = rbd(f0), rbd(f1), rbd(out)
            m = out["matches"]
            xy_i = f0["keypoints"][m[:, 0]].cpu().numpy().astype(np.float64)
            xy_j = f1["keypoints"][m[:, 1]].cpu().numpy().astype(np.float64)
        if len(xy_i) < 3:
            return None
        return xy_i[:max_matches], xy_j[:max_matches]


# --------------------------------------------------------------------------- #
# MASt3R (CC-BY-NC) — RESEARCH ORACLE ONLY                                     #
# --------------------------------------------------------------------------- #
class Mast3rMatcher:
    """MASt3R dense matcher. Returns 2D correspondences; we still lift them to
    metric 3D with our own depth (so MASt3R's metric pointmap is not relied on),
    keeping the pipeline identical across backends. Research/eval only."""

    _LONG_EDGE = 512  # model's expected long-edge size
    _PATCH_SIZE = 16  # ViT patch size; crop must be a multiple of this

    def __init__(self, device="cuda", weights=None):
        import torch

        _print_nc_banner("MASt3R", "CC-BY-NC-SA 4.0")
        self.torch = torch
        self.device = device
        from mast3r.model import AsymmetricMASt3R

        ckpt = weights or "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric"
        self.model = AsymmetricMASt3R.from_pretrained(ckpt).to(device).eval()

    def _prep(self, color_bgr, idx=0):
        """Prepare a BGR frame for MASt3R: resize long edge to 512, centre-crop
        to a multiple of patch_size (dust3r load_images() logic), ImgNorm. Return
        (view, (sx, sy)) where sx/sy scale descriptor-map (x, y) back to the
        original image resolution."""
        import numpy as np
        from dust3r.utils.image import ImgNorm, _resize_pil_image
        from PIL import Image

        pil = Image.fromarray(cv2_to_rgb(color_bgr))
        orig_W, orig_H = pil.size
        resized = _resize_pil_image(pil, self._LONG_EDGE)
        W_r, H_r = resized.size
        cx, cy = W_r // 2, H_r // 2
        halfw = ((2 * cx) // self._PATCH_SIZE) * self._PATCH_SIZE // 2
        halfh = ((2 * cy) // self._PATCH_SIZE) * self._PATCH_SIZE // 2
        cropped = resized.crop((cx - halfw, cy - halfh, cx + halfw, cy + halfh))
        W2, H2 = cropped.size
        img_t = ImgNorm(cropped)[None].to(self.device)
        view = {
            "img": img_t,
            "true_shape": np.int32([cropped.size[::-1]]),  # [H2, W2]
            "idx": idx,
            "instance": str(idx),
        }
        return view, (orig_W / W2, orig_H / H2)

    def match(self, fi, fj, max_matches=4000):
        import numpy as np
        import torch
        from dust3r.inference import inference
        from mast3r.fast_nn import fast_reciprocal_NNs

        with torch.inference_mode():
            v0, s0 = self._prep(fi.color, idx=0)
            v1, s1 = self._prep(fj.color, idx=1)
            out = inference(
                [(v0, v1)], self.model, self.device, batch_size=1, verbose=False
            )
            d0 = out["pred1"]["desc"].squeeze(0).detach()
            d1 = out["pred2"]["desc"].squeeze(0).detach()
            # fast_reciprocal_NNs returns (Nx2, Nx2) int64 (x, y) in crop space.
            xy0, xy1 = fast_reciprocal_NNs(
                d0, d1, subsample_or_initxy1=8, device=self.device
            )
        if len(xy0) < 3:
            return None
        xy_i = xy0.astype(np.float64) * np.array([[s0[0], s0[1]]])
        xy_j = xy1.astype(np.float64) * np.array([[s1[0], s1[1]]])
        return xy_i[:max_matches], xy_j[:max_matches]


# --------------------------------------------------------------------------- #
# MapAnything — apache checkpoint is commercial-safe; NC checkpoint = oracle    #
# --------------------------------------------------------------------------- #
class MapAnythingMatcher:
    """MapAnything pointmap matcher (#264).

    Unlike every other backend here, MapAnything is not a descriptor matcher:
    it regresses a per-view 3D **pointmap** for both views *into one shared
    frame*. Correspondence is therefore recovered geometrically — two pixels
    correspond when their regressed 3D points are mutual nearest neighbours in
    that shared frame — rather than photometrically.

    What that buys, and what it costs:
      + correspondences exist on texture-poor surfaces (blank walls) where
        descriptor matchers have nothing to key on;
      + the pair is reasoned about jointly, so a wide baseline is the regime
        the model was trained for.
      - a mutual-NN pair is only as trustworthy as the pointmap alignment. If
        the model puts the two views in the wrong relative place, it does so
        *smoothly*, and the resulting matches are dense, self-consistent and
        wrong — the exact failure mode PCM had to reject for cross-camera ORB
        in #236. The distance gate + reciprocity below are the only defence at
        this level; the RANSAC and PCM stages downstream are the real one.

    We deliberately discard MapAnything's own metric poses and re-lift the 2D
    matches through our stored depth (as every backend does), so relative poses
    stay comparable across matchers. Inference is image-only: no intrinsics,
    depth or pose is fed in, which keeps this an honest measurement of what the
    model alone contributes.

    The apache and NC checkpoints share this code path verbatim; only the
    checkpoint id resolved in __init__ differs.
    """

    _RESOLUTION_SET = 518  # resolution grid both checkpoints were trained on
    _STRIDE = 4  # pixel stride of the sampled pointmap grid
    _CONF_PERCENTILE = 20  # drop this % least-confident pixels per view
    _ABS_GATE_M = 0.05  # floor of the mutual-NN distance gate (metres)
    _SPACING_MULT = 1.5  # gate also scales with the map's own sample spacing
    _SPREAD_CELLS = 24  # keep <=1 match per (W/24)-wide cell of view i

    def __init__(self, device="cuda", weights=None, variant="apache"):
        import torch

        if variant != "apache":
            _print_nc_banner("MapAnything (13-dataset)", "CC-BY-NC 4.0")
        self.torch = torch
        self.device = device
        self.variant = variant
        from mapanything.models import MapAnything

        ckpt = weights or (
            "facebook/map-anything-apache"
            if variant == "apache"
            else "facebook/map-anything"
        )
        self.model = MapAnything.from_pretrained(ckpt).to(device).eval()
        # Per-call diagnostics for the visualiser (see visualize_matches.py).
        # match() keeps the (xy_i, xy_j) contract; this carries the extras.
        self.last_match_info = None

    # -- preprocessing ------------------------------------------------------ #
    def _prep(self, color_bgr, idx):
        """BGR frame -> a MapAnything view dict + the (sx, sy) that map resized
        pixel coords back to the original image.

        We resize the WHOLE image to the nearest trained resolution instead of
        using the upstream `crop_resize_if_necessary` centre-crop: on a
        wide-baseline pair the overlap is often at the image border, and
        cropping it away is exactly the signal we came for. The residual aspect
        distortion is small (the target is the closest trained aspect ratio) and
        harmless here — the model predicts per-pixel ray directions, and the
        metric lift downstream uses OUR intrinsics on the ORIGINAL image, not
        the model's.
        """
        import cv2
        import numpy as np
        import torch
        from mapanything.utils.image import find_closest_aspect_ratio
        from uniception.models.encoders.image_normalizations import (
            IMAGE_NORMALIZATION_DICT,
        )

        rgb = cv2_to_rgb(color_bgr)
        H, W = rgb.shape[:2]
        W2, H2 = find_closest_aspect_ratio(W / H, self._RESOLUTION_SET)
        interp = cv2.INTER_AREA if W2 < W else cv2.INTER_CUBIC
        small = cv2.resize(rgb, (W2, H2), interpolation=interp)

        norm = IMAGE_NORMALIZATION_DICT["dinov2"]
        t = torch.from_numpy(small).to(self.device).permute(2, 0, 1).float() / 255.0
        mean = norm.mean.view(3, 1, 1).to(self.device)
        std = norm.std.view(3, 1, 1).to(self.device)
        view = {
            "img": ((t - mean) / std)[None],
            "data_norm_type": ["dinov2"],
            "true_shape": np.int32([[H2, W2]]),
            "idx": idx,
            "instance": str(idx),
        }
        return view, (W / W2, H / H2)

    def _samples(self, pred):
        """Strided, masked, confidence-gated pointmap samples for one view.
        Returns (Nx3 world points, Nx2 resized-grid (u, v), N confidences)."""
        import numpy as np

        pts = pred["pts3d"][0].float().cpu().numpy()  # (H, W, 3), shared frame
        mask = pred["mask"][0, ..., 0].cpu().numpy().astype(bool)
        conf = pred["conf"][0].float().cpu().numpy()
        h, w = mask.shape
        s = self._STRIDE
        vv, uu = np.mgrid[0:h:s, 0:w:s]
        p = pts[vv, uu]
        m = mask[vv, uu] & np.isfinite(p).all(axis=-1)
        c = conf[vv, uu]
        if int(m.sum()) >= 16:
            m &= c >= np.percentile(c[m], self._CONF_PERCENTILE)
        uv = np.stack([uu[m], vv[m]], axis=1).astype(np.float64)
        return p[m], uv, c[m]

    def _spread(self, idx, uv, shape):
        """Thin a residual-sorted match list to <=1 per image cell of view i.

        Mutual-NN survivors CLUSTER. Where the two pointmaps carry a small
        systematic offset relative to each other, reciprocity only survives near
        the stationary points of that offset field, so the accepted matches
        bunch into a few patches. Feeding a bunched point set to RANSAC-Kabsch
        is a degenerate fit: it reports a healthy inlier count while the
        rotation is essentially unconstrained (measured on the office scan:
        matches with a 56-107 px spread gave 90 deg pose errors at 100+
        inliers, against ORB's 130-211 px spread). Spreading trades raw count
        for conditioning, which is what the pose solve actually needs.

        Input must already be sorted best-first; the first match reaching a cell
        wins it.
        """
        import numpy as np

        h, w = int(shape[0]), int(shape[1])
        cell = max(1, int(np.ceil(max(h, w) / self._SPREAD_CELLS)))
        keys = (uv[idx, 1].astype(np.int64) // cell) * (w // cell + 1) + (
            uv[idx, 0].astype(np.int64) // cell
        )
        _, first = np.unique(keys, return_index=True)
        return idx[np.sort(first)]

    @staticmethod
    def _to_full_res(uv, scale):
        """Resized-grid pixel coords -> original-image pixel coords, using
        cv2.resize's pixel-centre convention (x_src = (x_dst + .5)*s - .5)."""
        import numpy as np

        return (uv + 0.5) * np.asarray(scale, np.float64) - 0.5

    # -- the contract ------------------------------------------------------- #
    def match(self, fi, fj, max_matches=4000):
        import numpy as np
        import torch
        from scipy.spatial import cKDTree

        self.last_match_info = None
        v0, s0 = self._prep(fi.color, 0)
        v1, s1 = self._prep(fj.color, 1)
        with torch.inference_mode():
            preds = self.model.infer(
                [v0, v1],
                memory_efficient_inference=False,
                use_amp=True,
                amp_dtype="bf16",
                apply_mask=True,
                mask_edges=True,
            )
        p0, uv0, c0 = self._samples(preds[0])
        p1, uv1, c1 = self._samples(preds[1])
        if len(p0) < 3 or len(p1) < 3:
            return None

        # Mutual nearest 3D neighbour in the shared frame. Reciprocity kills the
        # many-to-one collapse you get where one view sees a surface at grazing
        # incidence and the other head-on.
        t1, t0 = cKDTree(p1), cKDTree(p0)
        d01, i01 = t1.query(p0, workers=-1)
        _, i10 = t0.query(p1, workers=-1)
        mutual = i10[i01] == np.arange(len(p0))

        # Distance gate. A pointmap is sampled on a pixel grid, so even a
        # perfect correspondence lands ~half a sample spacing away from its
        # partner; gating tighter than the grid would reject everything. Scale
        # the gate with the target map's own median spacing, with an absolute
        # floor for close-range pairs where the spacing is millimetric.
        spacing = float(np.median(t1.query(p1, k=2, workers=-1)[0][:, 1]))
        gate = max(self._ABS_GATE_M, self._SPACING_MULT * spacing)
        keep = mutual & np.isfinite(d01) & (d01 <= gate)
        n_keep = int(keep.sum())
        if n_keep < 3:
            return None

        idx = np.flatnonzero(keep)
        idx = idx[np.argsort(d01[idx])]  # best residual first
        idx = self._spread(idx, uv0, v0["true_shape"][0])[:max_matches]
        xy_i = self._to_full_res(uv0[idx], s0)
        xy_j = self._to_full_res(uv1[i01[idx]], s1)
        self.last_match_info = {
            "residual_m": d01[idx],  # mutual-NN 3D distance, metres
            "conf_i": c0[idx],
            "conf_j": c1[i01[idx]],
            "gate_m": gate,
            "spacing_m": spacing,
            "n_sampled": int(len(p0)),
            "n_mutual": int(mutual.sum()),
            "n_gated": n_keep,
        }
        return xy_i, xy_j


# --------------------------------------------------------------------------- #
def _print_nc_banner(model, licence):
    import sys

    print(
        f"\n{'='*72}\n[NON-COMMERCIAL] {model} weights are {licence}.\n"
        "Using them as an OFFLINE ACCURACY-CEILING ORACLE only. The resulting\n"
        "edge file is an evaluation artefact and MUST NOT be shipped in a\n"
        f"commercial deliverable.\n{'='*72}\n",
        file=sys.stderr,
    )


def cv2_to_rgb(color_bgr):
    import cv2

    return cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
