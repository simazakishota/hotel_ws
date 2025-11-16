#!/usr/bin/env python3
#!/usr/bin/env python3
import os
import sys
import open3d as o3d
import numpy as np
import torch

# ===== GraspNet モジュールパス追加 =====
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GRASPNET_BASE_PATH = os.path.expanduser(os.path.join(BASE_DIR, '../../../external/graspnet-baseline'))
GRASPNET_API_PATH  = os.path.expanduser(os.path.join(BASE_DIR, '../../../external/graspnetAPI'))

sys.path.append(GRASPNET_BASE_PATH)
sys.path.append(GRASPNET_API_PATH)

from models.graspnet import GraspNet
from graspnetAPI import GraspGroup
from graspnetAPI.grasp import Grasp


os.environ['CUDA_VISIBLE_DEVICES'] = ''  # GPU無効化

def main():
    filename = "/home/araishogo/hotel_ws/src/pointcloud_capture.ply"
    pcd = o3d.io.read_point_cloud(filename)
    print("✅ 点群読込完了:", len(pcd.points), "点")

    # ===== 地面平面検出 =====
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane_model

    pcd_no_ground = pcd.select_by_index(inliers, invert=True)
    pcd_ground    = pcd.select_by_index(inliers, invert=False)

    print(f"🌍 地面点群: {len(pcd_ground.points)} 点")
    print(f"✅ 地面除去後（物体）: {len(pcd_no_ground.points)} 点")
    print(f"🟫 検出された地面平面方程式: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")


    # ===== クラスタリング =====
    labels = np.array(pcd_no_ground.cluster_dbscan(eps=0.1, min_points=100, print_progress=True))
    if labels.max() < 0:
        raise RuntimeError("❌ クラスタが検出されませんでした。")
    counts = np.bincount(labels[labels >= 0])
    largest_idx = np.where(labels == np.argmax(counts))[0]
    largest_cluster = pcd_no_ground.select_by_index(largest_idx)
    print(f"✅ 最大クラスタ: 点数={len(largest_cluster.points)}")

    # ===== 最大クラスタ中心 =====
    cluster_center = np.asarray(largest_cluster.points).mean(axis=0)

    # ===== クラスタ周辺の地面だけ復活 =====
    ground_points = np.asarray(pcd_ground.points)
    dist_to_center = np.linalg.norm(ground_points - cluster_center, axis=1)

    # 20cm以内の地面だけ選ぶ
    local_ground_idx = np.where(dist_to_center < 0.3)[0]

    if len(local_ground_idx) == 0:
        print("⚠️ 注意: 周辺地面が見つからなかったため地面を 10 点だけ復活")
        # 近い地面10点だけ追加（たいてい十分）
        nearest_idx = np.argsort(dist_to_center)[:10]
        local_ground = pcd_ground.select_by_index(nearest_idx)
    else:
        local_ground = pcd_ground.select_by_index(local_ground_idx)


    print(f"🌏 クラスタ周辺20cm以内の地面: {len(local_ground.points)} 点")

    # ===== 地面法線 =====
    normal = np.array([a, b, c], dtype=np.float64)
    normal /= (np.linalg.norm(normal) + 1e-9)

    # ===== 最大クラスタの点群 =====
    obj = np.asarray(largest_cluster.points)

    # ===== 平面からの本当の距離 =====
    distances = np.abs(obj @ normal + d)   # ★これが正しい平面距離

    # ===== 高い方（距離が大きい方）から N 点を抜く =====
    N_TOP = 500
    idx_sorted = np.argsort(-distances)   # 大きい順
    top_idx = idx_sorted[:N_TOP]
    top_points = obj[top_idx]

    print(f"🟦 GraspNetに渡す上部点数: {len(top_points)} 点")

    # ===== GraspNetに渡す点群を numpy で構築 =====
    ground_np = np.asarray(local_ground.points)
    points_for_graspnet = np.vstack([top_points])

    print(f"🧩 GraspNet入力点群（修正後）: {len(points_for_graspnet)} 点")

    # ===== Open3D PointCloud に変換 =====
    pcd_for_graspnet = o3d.geometry.PointCloud()
    pcd_for_graspnet.points = o3d.utility.Vector3dVector(points_for_graspnet)




    # ===== GraspNetモデル読込 =====
    import os
    base_dir = os.path.expanduser('~/hotel_ws/external/graspnet-baseline')
    checkpoint_path = os.path.join(base_dir, 'checkpoint-rs.tar')

    net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                   cylinder_radius=0.05, is_training=False).to('cpu')
    state_dict = torch.load(checkpoint_path, map_location='cpu')
    net.load_state_dict(state_dict['model_state_dict'], strict=False)
    net.eval()
    print("✅ GraspNetモデル読込完了")

    # ===== 最大クラスタ点群で推論 =====
    points = np.asarray(pcd_for_graspnet.points)
    center_offset = points.mean(axis=0)
    points_centered = points - center_offset


    with torch.no_grad():
        end_points = {'point_clouds': torch.from_numpy(points_centered.astype(np.float32)).unsqueeze(0)}
        end_points = net(end_points)

    grasp_xyz  = end_points['vp_xyz'][0].cpu().numpy() + center_offset
    grasp_feat = end_points['vp_features'][0].cpu().numpy()
    score_pred = end_points['grasp_score_pred'][0].cpu().numpy()
    angle_pred = end_points['grasp_angle_cls_pred'][0].cpu().numpy()
    width_pred = end_points['grasp_width_pred'][0].cpu().numpy()

    # ===== 配列整形 =====
    grasp_score = score_pred.reshape(-1)
    grasp_angle = angle_pred.reshape(-1)
    grasp_width = width_pred.reshape(-1)

    num_points = grasp_xyz.shape[0]
    if grasp_score.shape[0] != num_points:
        min_len = min(grasp_score.shape[0], num_points)
        grasp_score = grasp_score[:min_len]
        grasp_angle = grasp_angle[:min_len]
        grasp_width = grasp_width[:min_len]
        grasp_xyz = grasp_xyz[:min_len]
        grasp_feat = grasp_feat[:min_len]
        print(f"⚙️ shape補正: {min_len} 点に統一")
    # ===== スコア補正 =====
    # ============================
    # ★ カメラ距離と grasp_score のみでスコア計算
    # ============================
    dist_camera = np.linalg.norm(grasp_xyz, axis=1)
    dist_camera_norm = dist_camera / (dist_camera.max() + 1e-6)

    score_norm = grasp_score / (grasp_score.max() + 1e-6)

    alpha = 0.7  # GraspNet スコア重み
    gamma = 0.3  # カメラ距離ペナルティ重み

    combined_score = (
        alpha * score_norm
        - gamma * dist_camera_norm
    )

    best_idx = int(np.argmax(combined_score))



    # ===== 地面法線 =====
    normal = np.array([a, b, c], dtype=np.float64)
    normal /= np.linalg.norm(normal)

    # ===== GraspNet が選んだ中心 =====
    best_center = grasp_xyz[best_idx].copy()


    # ===== 進入方向（best_approach）=====
    # ===== 進入方向（x軸）は地面に垂直 =====
    if normal[2] > 0:
        normal = -normal
    x_axis = -normal
    x_axis /= (np.linalg.norm(x_axis) + 1e-9)



    # ===== 局所点群（把持点周辺 3cm）を抽出 =====
    r_local = 0.05  # 3cm
    all_points = np.asarray(largest_cluster.points)
    d_local = np.linalg.norm(all_points - best_center, axis=1)
    local_patch = all_points[d_local < r_local]

    # 点が少ない場合は近い点を補う
    if len(local_patch) < 20:
        nearest_idx = np.argsort(d_local)[:50]
        local_patch = all_points[nearest_idx]

    # ===== PCA 法線推定 =====
    patch_mean = local_patch.mean(axis=0)
    X = local_patch - patch_mean
    cov = X.T @ X
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal_local = eigvecs[:, np.argmin(eigvals)]
    normal_local /= (np.linalg.norm(normal_local) + 1e-9)


    # ===== “上方向” を常に固定（地面に向かって -Z を下方向とする）=====
    ground_down = np.array([0, 0, -1.0])

    # PCA 法線と ground_down の向きを比較して 上方向 を決める
    if np.dot(normal_local, ground_down) < 0:
        up_dir = -ground_down    # 上（+Z）
    else:
        up_dir = ground_down     # 下（-Z）

    # 3cm の補正を加える
    best_center = best_center - up_dir * 0.03

    # ===== 直交軸生成 =====
     # ===== 把持点周辺の面法線（PCA法線）を z軸にする =====
    # ===== 軸の構築（y を PCA 法線、z = x × y）=====

    # y軸 = PCA 法線
    y_axis = normal_local
    y_axis /= (np.linalg.norm(y_axis) + 1e-9)

    # x と y が平行すぎないかチェック
    if abs(np.dot(x_axis, y_axis)) > 0.9:
        # 地面法線方向に少し混ぜる（安定化）
        y_axis = (normal_local + normal) / 2.0
        y_axis /= (np.linalg.norm(y_axis) + 1e-9)

    # z軸 = x × y（右手系）
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= (np.linalg.norm(z_axis) + 1e-9)

    # 最終姿勢行列
    R = np.stack([x_axis, y_axis, z_axis], axis=1)
    t = np.asarray(best_center, dtype=np.float64)


    # ===== Graspオブジェクト生成 =====
    width_m, height_m, depth_m = float(grasp_width[best_idx]), 0.02, 0.03
    g_best = Grasp(float(grasp_score[best_idx]), width_m, height_m, depth_m, R, t, np.array([0], dtype=np.int32))
    best_group = GraspGroup()
    best_group.grasp_group_array = np.vstack([best_group.grasp_group_array, g_best.grasp_array.reshape(1, -1)])

    # ===== 出力確認 =====
    print("🏆 最適把持候補（PCA面法線ベース）")
    print(f"  score={float(grasp_score[best_idx]):.4f}")
    print(f"  center={t}")
    print(f"  pca_normal={normal_local}")
    print("  R=\n", R)

    # ===== 可視化 =====
    largest_cluster.paint_uniform_color([0.0, 0.8, 1.0])
    local_ground.paint_uniform_color([0.6, 0.6, 0.6])  # 地面を灰色に表示
    geometries = [largest_cluster, local_ground] + best_group.to_open3d_geometry_list() 
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Best Grasp (Perpendicular to Ground Plane)",
        width=1000,
        height=800
    )
    # ===== 座標・姿勢を一時ファイルに保存 =====
    result_path = "/home/araishogo/hotel_ws/src/grasp_inference/grasp_result.npy"
    np.save(result_path, {'center': t, 'rotation': R})
    print(f"💾 Grasp結果を保存: {result_path}")

if __name__ == "__main__":
    main()
