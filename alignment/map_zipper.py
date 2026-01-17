import os
import copy
import yaml
import numpy as np
import open3d as o3d
from tqdm import tqdm

from utils.session import Session
from utils.session_map import SessionMap
from utils.logger import logger
from alignment.init_rt_finder import InitRTFinder
from alignment.matcher.open3d_scan_matcher import Open3DScanMatcher
#from alignment.matcher.pygicp_scan_matcher import PyGICPScanMatcher
from alignment.global_registration import register_with_fpfh_ransac


class MapZipper:
    def __init__(
        self,
        config_path: str,
    ):
        # Load parameters and initialize data loaders
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)
            
        #self.matcher_cls = Open3DScanMatcher
        matcher_name = self.params["alignment"].get("matcher", "Open3DScanMatcher")
        self.matcher_cls = {"Open3DScanMatcher": Open3DScanMatcher,
                            "PyGICPScanMatcher": PyGICPScanMatcher,
                            }.get(matcher_name, None)
        
        self.source_session = None
        self.tgt_session_map = None

    def load_target_session_map(self, tgt_session_map: SessionMap=None):
        p_settings = self.params["settings"]
        prev_output_dir = p_settings["prev_output_dir"]

        if tgt_session_map is None:
            logger.debug(f"Loading target session map from {prev_output_dir}")
            try:
                tgt_session_map = SessionMap()
                tgt_session_map.load(prev_output_dir, is_global=True)
            except FileNotFoundError:
                raise FileNotFoundError(f"Cannot find {prev_output_dir}")
        else:
            if not isinstance(tgt_session_map, SessionMap):
                raise TypeError("tgt_session_map must be an instance of SessionMap")
        self.tgt_session_map = tgt_session_map
    
    def load_source_session(self, src_session: Session=None):
        p_settings = self.params["settings"]
        if src_session is None:
            logger.debug(f"Loading source session from {p_settings['scans_dir']} and {p_settings['poses_file']}")
            try:
                src_session = Session(p_settings["scans_dir"], p_settings["poses_file"])
            except FileNotFoundError:
                raise FileNotFoundError(f"Cannot find {p_settings['scans_dir']} or {p_settings['poses_file']}")
        else:
            if not isinstance(src_session, Session):
                raise TypeError("src_session must be an instance of Session")
        self.src_session = src_session

    def _crop(self, pcd: o3d.geometry.PointCloud, center: np.ndarray, radius: float):
        min_b = center - radius
        max_b = center + radius
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_b, max_b)
        return pcd.crop(aabb)

    def _init_transform(self) -> np.ndarray:
        p_align = self.params["alignment"]
        p_settings = self.params["settings"]
        
        init_tf = np.array(p_align.get("init_transform", np.eye(4))).reshape(4, 4)
        is_identity = np.allclose(init_tf, np.eye(4))

        if not is_identity:
            return init_tf

        prev_out = p_settings.get("prev_output_dir", "")
        if prev_out and os.path.exists(prev_out):
            logger.info("Cannot find config initTF. Executing Kiss-Matcher based initTF finder.")
            prev_scans = os.path.join(os.path.dirname(prev_out.rstrip('/')), "Scans")
            f_params = copy.deepcopy(self.params)
            f_params["settings"]["prev_scans_dir"] = prev_scans
            
            try:
                finder = InitRTFinder(f_params)
                return finder.find_initial_transform()
            except Exception as e:
                logger.error(f"InitRTFinder Error: {e}")
        logger.warning("Failed to find initTF: use identity instead.")
        return np.eye(4)

    def _is_nonoverlapping(self, point: np.ndarray, threshold: float) -> bool:
        # Nearest-neighbor distance test
        tgt_session_map_poses = self.tgt_session_map.get_poses()
        if len(tgt_session_map_poses) == 0:
            raise ValueError("Target session map has no poses.")
        positions = np.vstack([pose[:3, 3] for pose in tgt_session_map_poses])
        kdtree = o3d.geometry.KDTreeFlann()
        kdtree.set_matrix_data(positions.T)
        _, idx, _ = kdtree.search_knn_vector_3d(point, 1)
        return np.linalg.norm(positions[idx] - point) > threshold

    def _update_poses_from(self, start: int, transform: np.ndarray, reverse: bool):
        rng = (range(start, -1, -1) if reverse 
               else range(start, len(self.src_session)))
        for j in rng:
            pose = self.src_session.get_pose(j)
            self.src_session.update_pose(j, transform @ pose)

    def _save_iteration(self, idx: int, cloud: o3d.geometry.PointCloud, reverse: bool):
        p_settings = self.params["settings"]        
        suffix = "rev_" if reverse else ""
        os.makedirs(p_settings["output_dir"], exist_ok=True)
        os.makedirs(os.path.join(p_settings["output_dir"], "aligned_poses"), exist_ok=True)
        os.makedirs(os.path.join(p_settings["output_dir"], "aligned_scans"), exist_ok=True)

        # Save updated poses
        pose_path = os.path.join(
            os.path.join(p_settings["output_dir"], "aligned_poses"), f"{suffix}aft_idx_{idx:06d}.txt"
        )
        self.src_session.save_pose(pose_path)
        
        # Color & write cloud
        cloud.paint_uniform_color([0, 1, 0])
        scan_path = os.path.join(
            os.path.join(p_settings["output_dir"], "aligned_scans"), f"{suffix}{idx:06d}.pcd"
        )
        o3d.io.write_point_cloud(scan_path, cloud)

    def _save_final_transform(self, reverse: bool):
        p_settings = self.params["settings"]        
        os.makedirs(p_settings["output_dir"], exist_ok=True)
    
        suffix = "rev_" if reverse else ""
        final_path = os.path.join(
            p_settings["output_dir"], f"{suffix}final_transform.txt"
        )
        self.src_session.save_pose(final_path)

    def _run_direction(self, reverse: bool):
        p_settings = self.params["settings"]
        p_alignment = self.params["alignment"]
        p = {**p_settings, **p_alignment}
        
        # forward: compute & apply init; reverse: reload forward-final
        if not reverse:
            init_tf = self._init_transform()
            logger.debug(f"Initial TF:\n{init_tf}")
            for i in range(len(self.src_session)):
                self.src_session.update_pose(
                    i,
                    init_tf @ self.src_session.get_pose(i)
                )
            init_path = os.path.join(p["output_dir"], "init_transform.txt")
            self.src_session.save_pose(init_path)
            logger.info(f"Saved init transform to {init_path}")
        else:
            final_path = os.path.join(p["output_dir"], "final_transform.txt")
            if not os.path.exists(final_path):
                raise RuntimeError(f"Cannot run reverse: missing {final_path}")
            self.src_session = Session(p["scans_dir"], final_path)
            logger.info(f"Loaded forward-final from {final_path}")

        # pre-merge target once
        merged_tgt = self.tgt_session_map.get()
        merged_tgt = merged_tgt.voxel_down_sample(p["tgt_voxel_size"])

        # scan loop
        desc = "Forward" if not reverse else "Reverse"
        logger.info(f"Running {'reverse' if reverse else 'forward'} pass")
        logger.debug(f"Using matcher: {self.matcher_cls.__name__}")
        idxs = range(len(self.src_session))
        idxs = [i for i in reversed(idxs)] if reverse else idxs
        for i in tqdm(idxs, desc=f"Alignment ({desc})", ncols=100):
            src_pc = self.src_session[i].downsample(p["src_voxel_size"]).get()
            query = self.src_session.get_pose(i)[:3, 3]
            tgt_crop = self._crop(merged_tgt, query, p.get("crop_radius", 100.0))

            if self._is_nonoverlapping(query, p.get("non_overlap_threshold", 10.0)):
                logger.debug(f"Scan {i} non-overlapping; skipping")
                tf_pc = copy.deepcopy(src_pc)
            else:
                maxd = p["gicp_max_correspondence_distance"]
                if reverse:
                    maxd /= 2.0
                matcher = self.matcher_cls(max_correspondence_distance=maxd)
                matcher.set_input_src(src_pc)
                matcher.set_input_tgt(tgt_crop)
                matcher.align()
                tf = matcher.get_final_transformation()
                tf_pc = copy.deepcopy(src_pc).transform(tf)
                self._update_poses_from(i, tf, reverse)
                fit, rmse = matcher.get_registration_result()
                logger.debug(f"Scan {i}: fit={fit:.2f}, rmse={rmse:.2f}")

            # self._save_iteration(i, tf_pc, reverse) # only for debug

        # write final
        self._save_final_transform(reverse)
        logger.info(f"{'Reverse' if reverse else 'Forward'} pass done.")

    def run(self):
        """Run both forward and then reverse registration."""
        p_settings = self.params["settings"]

        # if only one session is provided, return early
        if not self.tgt_session_map:
            self._save_final_transform(reverse=False)
            self._save_final_transform(reverse=True)
            return self.src_session
        
        # forward pass
        self._run_direction(reverse=False)
        # reverse pass
        self._run_direction(reverse=True)

        # return the updated session
        return Session(
            self.src_session.scans_dir,
            os.path.join(p_settings["output_dir"], "rev_final_transform.txt"),
        )


# Example usage:
if __name__ == "__main__":
    config = "./config/parkinglot.yaml"
    zipper = MapZipper(config)
    zipper.load_source_session()
    zipper.load_target_session_map()
    zipper.run()
    logger.info("Zipper finished.")  
