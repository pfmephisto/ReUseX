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

    def match(self, fi, fj, max_matches=4000):
        # MapAnything regresses per-view pointmaps in a shared frame; we convert
        # its dense correspondence field (nearest 3D neighbours between the two
        # pointmaps) into 2D matches, then re-lift with our own depth for a
        # consistent metric pose across all backends.
        raise NotImplementedError(
            "MapAnything correspondence extraction is wired up in the env-setup "
            "step (see tools/loop_edges/README.md); left as a clear seam so the "
            "commercial-safe apache checkpoint and the NC one share one code path."
        )


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
