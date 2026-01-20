import os
import copy
import warnings
import numpy as np
import open3d as o3d
from kiss_matcher import KISSMatcher
from utils.logger import logger

class InitRTFinder:
    def __init__(self, params: dict):
        self.params = params
        self.map_voxel_size = params["alignment"].get("tgt_voxel_size", 0.5)
        self.matcher_voxel_size = params["alignment"].get("matcher_voxel_size", 2.0)
        self.icp_voxel_size = params["alignment"].get("src_voxel_size", 0.15)
        p_set = params["settings"]
        self.prev_output_dir = p_set.get("prev_output_dir", "")
        if self.prev_output_dir:
            self.central_map_path = os.path.join(self.prev_output_dir, "lifelong_map.pcd")
        else:
            self.central_map_path = ""
        self.query_dir = p_set.get("scans_dir", "")
        self.query_pose_path = p_set.get("poses_file", "")
        self.query_start_idx = params["alignment"].get("query_start_idx", 0)

    def load_poses(self, path):
        poses = []
        if not os.path.exists(path):
            logger.error(f"Pose file not found: {path}")
            return poses
        with open(path) as f:
            for line in f:
                v = list(map(float, line.split()))
                T = np.eye(4)
                if len(v) == 12: T[:3, :] = np.array(v).reshape(3, 4)
                else: T = np.array(v).reshape(4, 4)
                poses.append(T)
        return poses

    def build_query_map(self, pcd_dir, poses, voxel):
        merged = o3d.geometry.PointCloud()
        for i in range(len(poses)):
            path = os.path.join(pcd_dir, f"{i:06d}.pcd")
            if not os.path.exists(path): continue
            pcd = o3d.io.read_point_cloud(path)
            pcd.transform(poses[i])
            if voxel > 0:
                pcd = pcd.voxel_down_sample(voxel)
            merged += pcd
            if i % 100 == 0:
                merged = merged.voxel_down_sample(voxel)
        return merged

    def crop_pcd_by_radius(self, pcd, center, radius):
        pts = np.asarray(pcd.points)
        if pts.size == 0: return pcd
        dists = np.linalg.norm(pts - center, axis=1)
        indices = np.where(dists < radius)[0]
        return pcd.select_by_index(indices)

    def solve_rt_svd(self, src_pts, tgt_pts):
        c_src, c_tgt = np.mean(src_pts, 0), np.mean(tgt_pts, 0)
        H = (src_pts - c_src).T @ (tgt_pts - c_tgt)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0: 
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, c_tgt - R @ c_src
        return T

    def find_initial_transform(self):
        warnings.filterwarnings("ignore", message=".*Too large.*noise_bound.*")
        if not os.path.exists(self.central_map_path):
            logger.error(f"Central map not found: {self.central_map_path}")
            return np.eye(4)
            
        c_map = o3d.io.read_point_cloud(self.central_map_path)
        if self.map_voxel_size > 0:
            c_map = c_map.voxel_down_sample(self.map_voxel_size)
            
        qp_poses = self.load_poses(self.query_pose_path)
        q_map = self.build_query_map(self.query_dir, qp_poses, self.map_voxel_size)

        bbox = c_map.get_axis_aligned_bounding_box()
        map_diag = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())
        CROP_RADIUS = (map_diag * 0.1) / 2.0
        logger.info(f"Radius-based initial alignment : radius={CROP_RADIUS:.2f}m")

        matcher = KISSMatcher(self.matcher_voxel_size)
        res = matcher.match(np.asarray(q_map.points).T, np.asarray(c_map.points).T)
        T_rel = self.solve_rt_svd(np.array(res[0]).reshape(-1, 3), np.array(res[1]).reshape(-1, 3))
        warnings.filterwarnings("default")

        q_origin_map = (qp_poses[self.query_start_idx] @ np.array([0, 0, 0, 1]))[:3]
        q_seg = self.crop_pcd_by_radius(q_map, q_origin_map, CROP_RADIUS)
        q_origin_in_central = (T_rel @ np.append(q_origin_map, 1.0))[:3]
        t_seg = self.crop_pcd_by_radius(c_map, q_origin_in_central, CROP_RADIUS)
        q_seg = q_seg.voxel_down_sample(self.icp_voxel_size) if self.icp_voxel_size > 0 else q_seg
        t_seg = t_seg.voxel_down_sample(self.icp_voxel_size) if self.icp_voxel_size > 0 else t_seg
        q_t = copy.deepcopy(q_seg).transform(T_rel)
        q_t.estimate_normals()
        t_seg.estimate_normals()
        reg = o3d.pipelines.registration.registration_icp(
            q_t, t_seg, 2.0, np.eye(4), 
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        final_T = reg.transformation @ T_rel
        logger.info(f"Radius-based initial alignment complete | fit: {reg.fitness:.4f} | rmse: {reg.inlier_rmse:.4f}")
        return final_T
