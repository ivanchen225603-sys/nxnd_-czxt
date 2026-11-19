"""供侧壁上沿高度和宽度共用的壁体归属识别。"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from carpointcloud_analyzer.domain.inner_faces import FACE_TOLERANCE_M
from carpointcloud_analyzer.domain.utils.parallel import stacked_svd


def side_body_mask(points: np.ndarray, side_mask: np.ndarray, edges: np.ndarray,
                   neighbor_count: int, *, inner_direction: int = 0) -> np.ndarray:
    """保留分段连续侧壁，排除独立杆件；返回本侧点掩码。"""
    side_mask = side_mask.copy()
    # P98 只能使用有局部几何支持的壁面/纵向顶沿点。沿用建系的邻点数与
    # 实测面 2.5cm 残差；按局部形状而非邻点距离筛选，保留稀疏、外折顶沿。
    # 排除待测点本身，并按坐标去重，避免一个点靠自身或重复补点通过验证。
    indices = np.flatnonzero(side_mask)
    side, inverse = np.unique(points[indices], axis=0, return_inverse=True)
    if len(side) >= 4:
        neighbors = cKDTree(side).query(side, k=min(neighbor_count + 1, len(side)), workers=-1)[1][:, 1:]
        samples = side[neighbors]
        centers = samples.mean(axis=1)
        centered = samples - centers[:, None, :]
        _, _, axes = stacked_svd(centered)
        normals = axes[:, -1, :]
        residual = np.abs(np.einsum('ij,ij->i', side - centers, normals))
        support_residual = np.abs(np.einsum('ijk,ik->ij', centered, normals))
        del centered
        keep = (residual <= FACE_TOLERANCE_M) & (np.percentile(support_residual, 90, axis=1) <= FACE_TOLERANCE_M)
        # 局部四点可确定线/面形状；纵向细顶沿沿用杆形支持的一维性与方向条件，
        # 不要求整条上沿都落在同一块竖直主壁面上。
        local = samples[:, :4, :]
        local_centers = local.mean(axis=1)
        _, singular, axes = stacked_svd(local - local_centers[:, None, :])
        directions = axes[:, 0, :]
        line_distance = np.linalg.norm(np.cross(side - local_centers, directions), axis=1)
        linearity_ratio, longitudinal_cosine = .35, .9
        linear = ((singular[:, 1] ** 2 < linearity_ratio * singular[:, 0] ** 2)
                  & (np.abs(directions[:, 1]) > longitudinal_cosine))
        keep |= linear & (line_distance <= FACE_TOLERANCE_M)
        # 局部形状成立不等于属于侧壁。沿用同一近邻图，只连接互为近邻的点，
        # 避免密集独立杆件单向吸引稀疏壁面；每段取高度覆盖最大的连续壁体。
        local_neighbors = neighbors
        rows = np.repeat(np.arange(len(side)), local_neighbors.shape[1])
        graph = csr_matrix((np.ones(len(rows), dtype=bool), (rows, local_neighbors.ravel())),
                           shape=(len(side), len(side)))
        graph = graph.minimum(graph.T).tocsr()
        # 线状点不能通过跨越空隙的长边接到墙上；相接的细上沿仍保留。
        # 这里使用线形判定的四点邻域尺度，壁面内部仍用完整邻域，避免
        # 各向异性扫描把同一块壁面误拆成互不相接的扫描线。
        local_radius = np.max(np.linalg.norm(local - side[:, None, :], axis=2), axis=1)
        # 近处多条绳索会进入四点邻域，看起来像一块面。先用点自身与前后两个
        # 最近邻确认纵向线，再分别看沿线采样和跨线采样，不能用拟合面的远邻跨空隙。
        line_samples = np.concatenate((side[:, None, :], samples[:, :2, :]), axis=1)
        line_centers = line_samples.mean(axis=1)
        _, line_singular, line_axes = stacked_svd(line_samples - line_centers[:, None, :])
        line_directions = line_axes[:, 0, :]
        line_like = ((line_singular[:, 1] ** 2 < linearity_ratio * line_singular[:, 0] ** 2)
                     & (np.abs(line_directions[:, 1]) > longitudinal_cosine))
        offsets = samples - side[:, None, :]
        line_like &= np.einsum('ij,ij->i', offsets[:, 0], offsets[:, 1]) < 0
        slope = np.divide(line_directions[:, 2], line_directions[:, 1],
                          out=np.zeros(len(side)), where=line_like)
        # 消除纵向倾斜后，最近的非同层观测给出本点跨高度层的采样间距。
        # 用高度差而非三维距离：真实内/外折沿的横向弯折不应被当成竖向空隙。
        layer_gap = np.abs(offsets[:, :, 2] - offsets[:, :, 1] * slope[:, None])
        layer_radius = np.min(np.where(layer_gap > FACE_TOLERANCE_M, layer_gap, np.inf), axis=1)
        layer_radius[~np.isfinite(layer_radius)] = 0.
        # 等间距扫描行经刚体变换后会有浮点舍入差；只补运算精度，不补测量距离。
        layer_radius += np.finfo(float).eps * np.maximum(1., np.linalg.norm(side, axis=1)) * neighbor_count
        links = graph.tocoo()
        # 后续按段取无向连通分量；互为近邻的边只需验一次，跨段边不会参与判定。
        bins = np.searchsorted(edges, side[:, 1], side='right') - 1
        row, col = links.row, links.col
        used = ((row < col) & keep[row] & keep[col] & (bins[row] == bins[col])
                & (bins[row] >= 0) & (bins[row] < len(edges) - 1))
        row, col = row[used], col[used]
        delta = side[col] - side[row]
        lengths = np.linalg.norm(delta, axis=1)
        connected = (~linear[row] | (lengths <= local_radius[row]))
        connected &= (~linear[col] | (lengths <= local_radius[col]))
        gap_from_row = np.abs(delta[:, 2] - delta[:, 1] * slope[row])
        gap_from_col = np.abs(delta[:, 2] - delta[:, 1] * slope[col])
        connected &= (~line_like[row] | (gap_from_row <= FACE_TOLERANCE_M)
                      | (gap_from_row <= layer_radius[row]))
        connected &= (~line_like[col] | (gap_from_col <= FACE_TOLERANCE_M)
                      | (gap_from_col <= layer_radius[col]))
        graph = csr_matrix((np.ones(connected.sum(), dtype=bool), (row[connected], col[connected])),
                           shape=graph.shape)
        body = np.zeros(len(side), dtype=bool)
        for index in range(len(edges) - 1):
            section = np.flatnonzero(keep & (bins == index))
            if not len(section):
                continue
            count, labels = connected_components(graph[section][:, section], directed=False)
            lower, upper = np.full(count, np.inf), np.full(count, -np.inf)
            np.minimum.at(lower, labels, side[section, 2])
            np.maximum.at(upper, labels, side[section, 2])
            span = upper - lower
            selected = int(np.argmax(span))
            if inner_direction:
                # 等高内外双层同时可见时，宽度量内侧上沿；高度调用保持原选择。
                candidates = np.flatnonzero(span == span[selected])
                selected = max(candidates, key=lambda label: inner_direction *
                               np.median(side[section[labels == label], 0]))
            body[section[labels == selected]] = True
        keep &= body
        side_mask[indices] = keep[inverse]
    return side_mask
