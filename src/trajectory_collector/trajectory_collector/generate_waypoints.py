# Copyright 2024 Clearpath Robotics / trajectory_collector contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generates reachable, obstacle-cleared start and goal waypoint pairs from a ROS 2 static map."""

import argparse
import csv
import math
import os
import random
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import cv2
import numpy as np
import yaml


def resolve_map_path(map_name_or_path: str) -> str:
    """Resolve map YAML from preset name or direct file path."""
    candidate = os.path.expanduser(map_name_or_path)

    # 1. Direct file path check
    if os.path.isfile(candidate):
        return os.path.abspath(candidate)

    # 2. Look up preset in clearpath_nav2_demos
    base_name = os.path.basename(candidate)
    if not base_name.endswith('.yaml'):
        base_name += '.yaml'

    try:
        demos_share = get_package_share_directory('clearpath_nav2_demos')
        pkg_map = os.path.join(demos_share, 'maps', base_name)
        if os.path.isfile(pkg_map):
            return os.path.abspath(pkg_map)
    except PackageNotFoundError:
        pass

    # 3. Fallback check inside local workspace directories
    ws_fallback = os.path.expanduser(
        os.path.join('~/husky_ws/install/clearpath_nav2_demos/share/clearpath_nav2_demos/maps',
                     base_name)
    )
    if os.path.isfile(ws_fallback):
        return os.path.abspath(ws_fallback)

    raise FileNotFoundError(
        f"Could not resolve map '{map_name_or_path}'. Provide a valid filepath or a known preset."
    )


def load_map(yaml_path: str):
    """Load map YAML metadata and grayscale image array."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f'Map YAML not found: {yaml_path}')

    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_rel = meta.get('image', '')
    image_path = image_rel if os.path.isabs(image_rel) else os.path.join(map_dir, image_rel)

    if not os.path.isfile(image_path):
        raise FileNotFoundError(f'Map image file not found: {image_path}')

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f'Could not read map image from {image_path}')

    resolution = float(meta['resolution'])
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    origin_x, origin_y = float(origin[0]), float(origin[1])
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    return (
        img,
        meta,
        resolution,
        (origin_x, origin_y, origin_yaw),
        negate,
        occupied_thresh,
        free_thresh
    )


def grid_to_world(col: int, row: int, height: int, resolution: float, origin: tuple) -> tuple:
    """Convert grid image coordinates (col, row) to world coordinates (x, y)."""
    origin_x, origin_y, origin_yaw = origin
    grid_x = (col + 0.5) * resolution
    grid_y = (height - 1 - row + 0.5) * resolution

    if abs(origin_yaw) > 1e-6:
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        world_x = c * grid_x - s * grid_y + origin_x
        world_y = s * grid_x + c * grid_y + origin_y
    else:
        world_x = grid_x + origin_x
        world_y = grid_y + origin_y

    return world_x, world_y


def is_pair_dissimilar(new_pair: tuple, existing_pairs: list, thresh: float) -> bool:
    """Check if candidate pair is sufficiently separated from existing pairs."""
    sx1, sy1, gx1, gy1 = new_pair
    for ex in existing_pairs:
        sx2, sy2, gx2, gy2 = ex
        # Direct similarity check
        d_start = math.hypot(sx1 - sx2, sy1 - sy2)
        d_goal = math.hypot(gx1 - gx2, gy1 - gy2)
        if d_start < thresh and d_goal < thresh:
            return False
        # Reverse similarity check (avoid sampling exact reverse path)
        d_rev_start = math.hypot(sx1 - gx2, sy1 - gy2)
        d_rev_goal = math.hypot(gx1 - sx2, gy1 - sy2)
        if d_rev_start < thresh and d_rev_goal < thresh:
            return False
    return True


def generate_waypoints(
    map_input: str = 'warehouse',
    num_samples: int = 10,
    min_distance: float = 4.0,
    max_distance: float = 25.0,
    clearance_m: float = 0.65,
    similarity_thresh: float = 2.0,
    face_goal: bool = True,
    seed: int = None,
    preview_path: str = ''
) -> list:
    """Generate topologically reachable, clearance-verified waypoint pairs."""
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    map_yaml = resolve_map_path(map_input)
    img, meta, res, origin, negate, occ_th, free_th = load_map(map_yaml)
    height, width = img.shape

    # 1. Compute occupancy probability
    if negate == 0:
        occ = (255.0 - img) / 255.0
    else:
        occ = img / 255.0

    free_mask = (occ < free_th).astype(np.uint8) * 255

    # 2. Morphological erosion to enforce robot clearance
    clearance_px = max(1, int(math.ceil(clearance_m / res)))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (clearance_px * 2 + 1, clearance_px * 2 + 1)
    )
    eroded_mask = cv2.erode(free_mask, kernel)

    # 3. Connected components to ensure all sampled points are mutually reachable
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded_mask)
    if num_labels <= 1:
        raise RuntimeError(
            f'No free space remaining on map after {clearance_m}m clearance erosion!'
        )

    # Largest connected component (ignoring background label 0)
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_label = int(np.argmax(areas) + 1)

    valid_rows, valid_cols = np.where(labels == largest_label)
    num_valid = len(valid_rows)
    if num_valid < 2:
        raise RuntimeError('Insufficient navigable cells in the largest connected component.')

    # 4. Sample start-goal pairs
    sampled_pairs = []
    candidates = []
    max_attempts = num_samples * 1000
    attempts = 0
    map_tag = os.path.splitext(os.path.basename(map_yaml))[0]

    while len(sampled_pairs) < num_samples and attempts < max_attempts:
        attempts += 1
        i_start = random.randint(0, num_valid - 1)
        i_goal = random.randint(0, num_valid - 1)
        if i_start == i_goal:
            continue

        r_s, c_s = valid_rows[i_start], valid_cols[i_start]
        r_g, c_g = valid_rows[i_goal], valid_cols[i_goal]

        sx, sy = grid_to_world(c_s, r_s, height, res, origin)
        gx, gy = grid_to_world(c_g, r_g, height, res, origin)

        dist = math.hypot(gx - sx, gy - sy)
        if dist < min_distance or dist > max_distance:
            continue

        pair_coords = (sx, sy, gx, gy)
        if not is_pair_dissimilar(pair_coords, candidates, similarity_thresh):
            continue

        candidates.append(pair_coords)

        # Yaw orientations
        if face_goal:
            rad = math.atan2(gy - sy, gx - sx)
            start_yaw_rad = rad
            start_yaw_deg = math.degrees(rad)
            goal_yaw_deg = start_yaw_deg
        else:
            deg = random.uniform(-180.0, 180.0)
            start_yaw_deg = deg
            start_yaw_rad = math.radians(deg)
            goal_yaw_deg = random.uniform(-180.0, 180.0)

        pair_dict = {
            'id': len(sampled_pairs),
            'start_x': round(sx, 3),
            'start_y': round(sy, 3),
            'start_yaw_rad': round(start_yaw_rad, 4),
            'start_yaw_deg': round(start_yaw_deg, 2),
            'goal_x': round(gx, 3),
            'goal_y': round(gy, 3),
            'goal_yaw_deg': round(goal_yaw_deg, 2),
            'distance': round(dist, 3),
            'map': map_tag,
            'start_pixel': (c_s, r_s),
            'goal_pixel': (c_g, r_g),
        }
        sampled_pairs.append(pair_dict)

    if len(sampled_pairs) < num_samples:
        print(
            f'Warning: Only generated {len(sampled_pairs)}/{num_samples} pairs '
            f'after {attempts} sampling attempts.',
            file=sys.stderr
        )

    # 5. Visual verification overlay image
    if preview_path:
        preview_full = os.path.expanduser(preview_path)
        os.makedirs(os.path.dirname(os.path.abspath(preview_full)) or '.', exist_ok=True)
        preview_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Faint cyan overlay for valid clearance area
        clearance_overlay = preview_rgb.copy()
        clearance_overlay[eroded_mask > 0] = [200, 160, 0]
        cv2.addWeighted(clearance_overlay, 0.3, preview_rgb, 0.7, 0, preview_rgb)

        for p in sampled_pairs:
            cs, rs = p['start_pixel']
            cg, rg = p['goal_pixel']
            # Draw route arrow from start to goal
            cv2.arrowedLine(preview_rgb, (cs, rs), (cg, rg), (0, 0, 220), 2, tipLength=0.03)
            # Draw start marker (green) and goal marker (blue)
            cv2.circle(preview_rgb, (cs, rs), 4, (0, 200, 0), -1)
            cv2.circle(preview_rgb, (cg, rg), 4, (220, 80, 0), -1)
            cv2.putText(
                preview_rgb, str(p['id']), (cs + 5, rs - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 140, 0), 1
            )

        cv2.imwrite(preview_full, preview_rgb)

    return sampled_pairs


def save_waypoints_to_csv(waypoints: list, output_path: str):
    """Save generated waypoints to a CSV file."""
    resolved_path = os.path.expanduser(output_path)
    os.makedirs(os.path.dirname(os.path.abspath(resolved_path)) or '.', exist_ok=True)

    fields = [
        'id',
        'start_x',
        'start_y',
        'start_yaw_rad',
        'start_yaw_deg',
        'goal_x',
        'goal_y',
        'goal_yaw_deg',
        'distance',
        'map'
    ]

    with open(resolved_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for wp in waypoints:
            row = {k: wp[k] for k in fields}
            writer.writerow(row)


def main():
    """CLI entry point for waypoint generation."""
    parser = argparse.ArgumentParser(
        description='Generate clearance-verified, reachable start-goal waypoints from map.'
    )
    parser.add_argument(
        '--map', '-m', default='warehouse',
        help="Map preset name (e.g. 'warehouse') or path to map YAML file (default: 'warehouse')"
    )
    parser.add_argument(
        '-n', '--num-samples', type=int, default=10,
        help='Number of waypoint pairs to generate (default: 10)'
    )
    parser.add_argument(
        '--min-dist', type=float, default=4.0,
        help='Minimum distance in meters between start and goal (default: 4.0)'
    )
    parser.add_argument(
        '--max-dist', type=float, default=25.0,
        help='Maximum distance in meters between start and goal (default: 25.0)'
    )
    parser.add_argument(
        '--clearance', type=float, default=0.65,
        help='Robot clearance radius in meters from obstacles (default: 0.65 for Husky A200)'
    )
    parser.add_argument(
        '--similarity-thresh', type=float, default=2.0,
        help='Minimum distance in meters between distinct sampled pairs (default: 2.0)'
    )
    parser.add_argument(
        '--seed', type=int, default=None,
        help='Optional random seed for reproducibility (default: None)'
    )
    parser.add_argument(
        '--random-yaw', action='store_true',
        help='Randomize start/goal yaw orientations instead of facing the goal direction'
    )
    parser.add_argument(
        '-o', '--output', default='data/waypoints.csv',
        help='Output CSV filepath (default: data/waypoints.csv)'
    )
    parser.add_argument(
        '--preview', default='',
        help='Optional filepath to save visual map overlay image (e.g. data/waypoints_preview.png)'
    )

    args = parser.parse_args()

    try:
        waypoints = generate_waypoints(
            map_input=args.map,
            num_samples=args.num_samples,
            min_distance=args.min_dist,
            max_distance=args.max_dist,
            clearance_m=args.clearance,
            similarity_thresh=args.similarity_thresh,
            face_goal=not args.random_yaw,
            seed=args.seed,
            preview_path=args.preview
        )
        save_waypoints_to_csv(waypoints, args.output)
        print(f'Successfully generated {len(waypoints)} waypoint pairs to: {args.output}')
        if args.preview:
            print(f'Visual map preview saved to: {args.preview}')
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
