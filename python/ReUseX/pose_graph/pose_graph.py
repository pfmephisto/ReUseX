from .._core import Dataset

import g2o.g2opy as g2o

import math
import numpy as np

import torch
from torch.nn import Module
from torch import Tensor
from torchvision.transforms import Compose, Normalize, ToTensor



from mast3r.model import AsymmetricMASt3R


from scipy.spatial import KDTree

from PIL import Image
from PIL.ImageOps import exif_transpose
import cv2


device = 'cuda' if torch.cuda.is_available() else 'cpu'
model_name = 'naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric'

# img = exif_transpose(image_1).convert("RGB")

def _resize_pil_image(img, long_edge_size):
    S = max(img.size)
    if S > long_edge_size:
        interp = Image.LANCZOS
    elif S <= long_edge_size:
        interp = Image.BICUBIC
    new_size = tuple(int(round(x * long_edge_size / S)) for x in img.size)
    return img.resize(new_size, interp)

def resize(img, size = 256, square_ok = False):
    W1, H1 = img.size
    if size == 224:
        # resize short side to 224 (then crop)
        img = _resize_pil_image(img, round(size * max(W1 / H1, H1 / W1)))
    else:
        # resize long side to 512
        img = _resize_pil_image(img, size)
    W, H = img.size
    cx, cy = W // 2, H // 2
    if size == 224:
        half = min(cx, cy)
        img = img.crop((cx - half, cy - half, cx + half, cy + half))
    else:
        halfw, halfh = ((2 * cx) // 16) * 8, ((2 * cy) // 16) * 8
        if not (square_ok) and W == H:
            halfh = 3 * halfw / 4
        img = img.crop((cx - halfw, cy - halfh, cx + halfw, cy + halfh))
    return img

def fast_reciprocal_NNs(pts1, pts2, subsample_or_initxy1=8, ret_xy=True, pixel_tol=0, ret_basin=False, device="cuda", **matcher_kw):
    slice_ = slice(1, None, 1) if len(pts1.shape) > 3 else slice(0, None, 1)
    H1, W1, DIM1 = pts1.shape[slice_]
    H2, W2, DIM2 = pts2.shape[slice_]
    assert DIM1 == DIM2

    pts1 = pts1.reshape(-1, DIM1)
    pts2 = pts2.reshape(-1, DIM2)

    if isinstance(subsample_or_initxy1, int) and pixel_tol == 0:
        S = subsample_or_initxy1
        y1, x1 = np.mgrid[S // 2 : H1 : S, S // 2 : W1 : S].reshape(2, -1)
        max_iter = 10
    else:
        x1, y1 = subsample_or_initxy1
        if isinstance(x1, torch.Tensor):
            x1 = x1.cpu().numpy()
        if isinstance(y1, torch.Tensor):
            y1 = y1.cpu().numpy()
        max_iter = 1

    xy1 = np.int32(np.unique(x1 + W1 * y1))  # make sure there's no doublons
    xy2 = np.full_like(xy1, -1)
    old_xy1 = xy1.copy()
    old_xy2 = xy2.copy()

    if (
        "dist" in matcher_kw
        or "block_size" in matcher_kw
        or (isinstance(device, str) and device.startswith("cuda"))
        or (isinstance(device, torch.device) and device.type.startswith("cuda"))
    ):
        pts1 = pts1.to(device)
        pts2 = pts2.to(device)
        tree1 = cdistMatcher(pts1, device=device)
        tree2 = cdistMatcher(pts2, device=device)
    else:
        pts1, pts2 = to_numpy((pts1, pts2))
        tree1 = KDTree(pts1)
        tree2 = KDTree(pts2)

    notyet = np.ones(len(xy1), dtype=bool)
    basin = np.full((H1 * W1 + 1,), -1, dtype=np.int32) if ret_basin else None

    niter = 0
    # n_notyet = [len(notyet)]
    while notyet.any():
        _, xy2[notyet] = to_numpy(tree2.query(pts1[xy1[notyet]], **matcher_kw))
        if not ret_basin:
            notyet &= old_xy2 != xy2  # remove points that have converged

        _, xy1[notyet] = to_numpy(tree1.query(pts2[xy2[notyet]], **matcher_kw))
        if ret_basin:
            basin[old_xy1[notyet]] = xy1[notyet]
        notyet &= old_xy1 != xy1  # remove points that have converged

        # n_notyet.append(notyet.sum())
        niter += 1
        if niter >= max_iter:
            break

        old_xy2[:] = xy2
        old_xy1[:] = xy1

    # print('notyet_stats:', ' '.join(map(str, (n_notyet+[0]*10)[:max_iter])))

    if pixel_tol > 0:
        # in case we only want to match some specific points
        # and still have some way of checking reciprocity
        old_yx1 = np.unravel_index(old_xy1, (H1, W1))[0].base
        new_yx1 = np.unravel_index(xy1, (H1, W1))[0].base
        dis = np.linalg.norm(old_yx1 - new_yx1, axis=-1)
        converged = dis < pixel_tol
        if not isinstance(subsample_or_initxy1, int):
            xy1 = old_xy1  # replace new points by old ones
    else:
        converged = ~notyet  # converged correspondences

    # keep only unique correspondences, and sort on xy1
    xy1, xy2 = merge_corres(xy1[converged], xy2[converged], (H1, W1), (H2, W2), ret_xy=ret_xy)
    if ret_basin:
        return xy1, xy2, basin
    return xy1, xy2

def todevice(batch, device, callback=None, non_blocking=False):
    """Transfer some variables to another device (i.e. GPU, CPU:torch, CPU:numpy).

    batch: list, tuple, dict of tensors or other things
    device: pytorch device or 'numpy'
    callback: function that would be called on every sub-elements.
    """
    if callback:
        batch = callback(batch)

    if isinstance(batch, dict):
        return {k: todevice(v, device) for k, v in batch.items()}

    if isinstance(batch, (tuple, list)):
        return type(batch)(todevice(x, device) for x in batch)

    x = batch
    if device == "numpy":
        if isinstance(x, torch.Tensor):
            x = x.detach().cpu().numpy()
    elif x is not None:
        if isinstance(x, np.ndarray):
            x = torch.from_numpy(x)
        if torch.is_tensor(x):
            x = x.to(device, non_blocking=non_blocking)
    return x


def to_numpy(x):
    return todevice(x, "numpy")

def merge_corres(idx1, idx2, shape1=None, shape2=None, ret_xy=True, ret_index=False):
    assert idx1.dtype == idx2.dtype == np.int32

    # unique and sort along idx1
    corres = np.unique(np.c_[idx2, idx1].view(np.int64), return_index=ret_index)
    if ret_index:
        corres, indices = corres
    xy2, xy1 = corres[:, None].view(np.int32).T

    if ret_xy:
        assert shape1 and shape2
        xy1 = np.unravel_index(xy1, shape1)
        xy2 = np.unravel_index(xy2, shape2)
        if ret_xy != "y_x":
            xy1 = xy1[0].base[:, ::-1]
            xy2 = xy2[0].base[:, ::-1]

    if ret_index:
        return xy1, xy2, indices
    return xy1, xy2

@torch.no_grad()
def bruteforce_reciprocal_nns(A, B, device="cuda", block_size=None, dist="l2"):
    if isinstance(A, np.ndarray):
        A = torch.from_numpy(A).to(device)
    if isinstance(B, np.ndarray):
        B = torch.from_numpy(B).to(device)

    A = A.to(device)
    B = B.to(device)

    if dist == "l2":
        dist_func = torch.cdist
        argmin = torch.min
    elif dist == "dot":

        def dist_func(A, B):
            return A @ B.T

        def argmin(X, dim):
            sim, nn = torch.max(X, dim=dim)
            return sim.neg_(), nn

    else:
        raise ValueError(f"Unknown {dist=}")

    if block_size is None or len(A) * len(B) <= block_size**2:
        dists = dist_func(A, B)
        _, nn_A = argmin(dists, dim=1)
        _, nn_B = argmin(dists, dim=0)
    else:
        dis_A = torch.full((A.shape[0],), float("inf"), device=device, dtype=A.dtype)
        dis_B = torch.full((B.shape[0],), float("inf"), device=device, dtype=B.dtype)
        nn_A = torch.full((A.shape[0],), -1, device=device, dtype=torch.int64)
        nn_B = torch.full((B.shape[0],), -1, device=device, dtype=torch.int64)
        number_of_iteration_A = math.ceil(A.shape[0] / block_size)
        number_of_iteration_B = math.ceil(B.shape[0] / block_size)

        for i in range(number_of_iteration_A):
            A_i = A[i * block_size : (i + 1) * block_size]
            for j in range(number_of_iteration_B):
                B_j = B[j * block_size : (j + 1) * block_size]
                dists_blk = dist_func(A_i, B_j)  # A, B, 1
                # dists_blk = dists[i * block_size:(i+1)*block_size, j * block_size:(j+1)*block_size]
                min_A_i, argmin_A_i = argmin(dists_blk, dim=1)
                min_B_j, argmin_B_j = argmin(dists_blk, dim=0)

                col_mask = min_A_i < dis_A[i * block_size : (i + 1) * block_size]
                line_mask = min_B_j < dis_B[j * block_size : (j + 1) * block_size]

                dis_A[i * block_size : (i + 1) * block_size][col_mask] = min_A_i[col_mask]
                dis_B[j * block_size : (j + 1) * block_size][line_mask] = min_B_j[line_mask]

                nn_A[i * block_size : (i + 1) * block_size][col_mask] = argmin_A_i[col_mask] + (j * block_size)
                nn_B[j * block_size : (j + 1) * block_size][line_mask] = argmin_B_j[line_mask] + (i * block_size)
    nn_A = nn_A.cpu().numpy()
    nn_B = nn_B.cpu().numpy()
    return nn_A, nn_B

class cdistMatcher:
    def __init__(self, db_pts, device="cuda"):
        self.db_pts = db_pts.to(device)
        self.device = device

    def query(self, queries, k=1, **kw):
        assert k == 1
        if queries.numel() == 0:
            return None, []
        nnA, nnB = bruteforce_reciprocal_nns(queries, self.db_pts, device=self.device, **kw)
        dis = None
        return dis, nnA


ImgNorm = Compose([ToTensor(), Normalize(mean=(0.5, 0.5, 0.5),std=(0.5, 0.5, 0.5))])

def create_pose_graph(dataset: Dataset, model: Module)-> g2o.SparseOptimizer:
    optimizer = g2o.SparseOptimizer()

    # Not planing on running the solver just creating the graph
    # solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
    # solver = g2o.OptimizationAlgorithmLevenberg(solver)
    # optimizer.set_algorithm(solver)
    # optimizer.set_verbose(True)


    model = AsymmetricMASt3R.from_pretrained(model_name).to(device)

    image_1 = dataset[0].image
    image_2 = dataset[1].image


    img = resize(image_1, 512)
    view1 = dict(img=ImgNorm(img).unsqueeze(0).to(device), true_shape=torch.from_numpy(np.int32([img.size[::-1]])),  idx=0, instance="0")
    img = ImgNorm(resize(image_2, 512)).unsqueeze(0).to(device)
    img = resize(image_2, 512)
    view2 = dict(img=ImgNorm(img).unsqueeze(0).to(device), true_shape=torch.from_numpy(np.int32([img.size[::-1]])), idx=1, instance="1")

    with torch.amp.autocast(device, enabled=bool(False)):
        pred1, pred2 = model(view1, view2)

    desc1, desc2 = pred1["desc"].squeeze(0).detach(), pred2["desc"].squeeze(0).detach()


    matches_im0, matches_im1 = fast_reciprocal_NNs(desc1, desc2, subsample_or_initxy1=8, device=device, dist="dot", block_size=2**13)

    # ignore small border around the edge
    H0, W0 = view1["true_shape"][0]
    valid_matches_im0 = (matches_im0[:, 0] >= 3) & (matches_im0[:, 0] < int(W0) - 3) & (matches_im0[:, 1] >= 3) & (matches_im0[:, 1] < int(H0) - 3)

    H1, W1 = view2["true_shape"][0]
    valid_matches_im1 = (matches_im1[:, 0] >= 3) & (matches_im1[:, 0] < int(W1) - 3) & (matches_im1[:, 1] >= 3) & (matches_im1[:, 1] < int(H1) - 3)

    valid_matches = valid_matches_im0 & valid_matches_im1

    # return valid_matches, matches_im0, matches_im1, view1, view2

    print(f"Found {valid_matches.sum()} matches")

    pts_1 = matches_im0[valid_matches].astype(np.float32)
    pts_2 = matches_im1[valid_matches].astype(np.float32)

    mat, mask = cv2.findHomography(pts_1, pts_2, cv2.RANSAC, 5.0) # 0, cv2.RANSAC,cv2.LMEDS,  cv2.RHO 

    # for idx, data in enumerate(dataset):
    #     pass
        

    return optimizer


def find_matches(view1:dict, view2:dict, model):

    # Fid match
    pred1, pred2 = model(view1, view2)

    desc1, desc2 = pred1["desc"].squeeze(0).detach(), pred2["desc"].squeeze(0).detach()


    matches_im0, matches_im1 = fast_reciprocal_NNs(desc1, desc2, subsample_or_initxy1=8, device=device, dist="dot", block_size=2**13)


    # ignore small border around the edge
    H0, W0 = view1["true_shape"][0]
    valid_matches_im0 = (matches_im0[:, 0] >= 3) & (matches_im0[:, 0] < int(W0) - 3) & (matches_im0[:, 1] >= 3) & (matches_im0[:, 1] < int(H0) - 3)

    H1, W1 = view2["true_shape"][0]
    valid_matches_im1 = (matches_im1[:, 0] >= 3) & (matches_im1[:, 0] < int(W1) - 3) & (matches_im1[:, 1] >= 3) & (matches_im1[:, 1] < int(H1) - 3)

    valid_matches = valid_matches_im0 & valid_matches_im1

    return valid_matches, matches_im0, matches_im1, view1, view2

