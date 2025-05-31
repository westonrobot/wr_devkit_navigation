#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
import time

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # パラメータの宣言と取得
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('marker_size', 0.7)  # C++コードと同じ0.1m
        self.declare_parameter('marker_id', 10)  # マーカーID
        self.declare_parameter('aruco_dict_id', 'DICT_6X6_50')  # デフォルトを6X6に変更

        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_marker_id = self.get_parameter('marker_id').value
        aruco_dict_id = self.get_parameter('aruco_dict_id').value
        
        # 最後にログを出力した時間を記録
        self.last_log_time = 0.0
        
        # AruCoディクショナリの設定 - C++コードと同等のマッピング
        aruco_dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL
        }
        self.aruco_dict = aruco.Dictionary_get(aruco_dict_map.get(aruco_dict_id, aruco.DICT_6X6_50))
        # C++と同様のシンプルな検出パラメータ
        self.aruco_params = aruco.DetectorParameters_create()
        
        # カメラ情報変数
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # OpenCVブリッジ
        self.bridge = CvBridge()
        
        # サブスクライバー
        self.image_subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)
        
        # パブリッシャー
        self.markers_publisher = self.create_publisher(
            Int32MultiArray, 
            'aruco_markers', 
            10)
        self.marker_poses_publisher = self.create_publisher(
            PoseStamped, 
            'marker_pose', 
            10)
        self.debug_image_publisher = self.create_publisher(
            Image, 
            'aruco_detection/image_raw', 
            10)
        
        self.get_logger().info(f'AruCo検出ノードを初期化しました。カメラトピック: {camera_topic}')
        
    def camera_info_callback(self, msg):
        """カメラ情報を受信したときのコールバック"""
        if self.camera_matrix is None:
            # カメラ行列の取得
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            # 歪み係数の取得
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('カメラパラメータを取得しました')
    
    def should_log_now(self):
        """現在ログを出力すべきかどうかを判断する"""
        current_time = time.time()
        if current_time - self.last_log_time >= 10.0:  # 10秒間隔
            self.last_log_time = current_time
            return True
        return False
            
    def image_callback(self, msg):
        """画像を受信したときのコールバック"""
        if self.camera_matrix is None:
            self.get_logger().warning('カメラパラメータがまだ利用できません')
            return
            
        try:
            # ROS画像メッセージをOpenCV形式に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # C++コード同様にシンプルな処理
            # マーカー検出 - 前処理を簡素化
            corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
            # 検出結果をデバッグ表示
            if self.should_log_now():
                self.get_logger().info(f"検出試行: 辞書タイプ={self.get_parameter('aruco_dict_id').value}, 対象ID={self.target_marker_id}")
                if ids is not None:
                    self.get_logger().info(f"検出マーカー: ids={ids.flatten().tolist()}")
                else:
                    self.get_logger().info("マーカーは検出されませんでした")
            
            # 検出結果をパブリッシュするためのメッセージ
            markers_msg = Int32MultiArray()
            
            # マーカーが検出された場合の処理 - C++コードと同等のロジック
            if ids is not None and len(corners) > 0:
                # 検出されたマーカーIDをリストに変換
                detected_ids = ids.flatten().tolist()
                markers_msg.data = detected_ids
                
                # 姿勢推定
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                # 目標とするマーカーIDが検出された場合
                if self.target_marker_id in detected_ids:
                    marker_index = detected_ids.index(self.target_marker_id)
                    
                    # マーカーの姿勢をパブリッシュ
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    
                    # 位置
                    pose_msg.pose.position.x = tvecs[marker_index][0][0]
                    pose_msg.pose.position.y = tvecs[marker_index][0][1]
                    pose_msg.pose.position.z = tvecs[marker_index][0][2]
                    
                    # 回転行列からクォータニオンに変換（シンプルに）
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[marker_index][0])
                    
                    # 回転行列をクォータニオンに変換
                    trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
                    if trace > 0:
                        s = 0.5 / np.sqrt(trace + 1.0)
                        w = 0.25 / s
                        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
                        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
                        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
                    else:
                        if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                            s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                            w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                            x = 0.25 * s
                            y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                            z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                            s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                            w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                            x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                            y = 0.25 * s
                            z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                        else:
                            s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                            w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                            x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                            y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                            z = 0.25 * s
                    
                    # クォータニオンをセット
                    pose_msg.pose.orientation.w = w
                    pose_msg.pose.orientation.x = x
                    pose_msg.pose.orientation.y = y
                    pose_msg.pose.orientation.z = z
                    
                    self.marker_poses_publisher.publish(pose_msg)
                    
                    # 検出結果の可視化 - ここをC++コードのシンプルさと合わせる
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    
                    # 座標軸を描画
                    for i in range(len(ids)):
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, 
                                        rvecs[i], tvecs[i], self.marker_size/2)
                    
                    if self.should_log_now():
                        self.get_logger().info(f'マーカーID {self.target_marker_id} を検出：位置 [{tvecs[marker_index][0][0]:.3f}, {tvecs[marker_index][0][1]:.3f}, {tvecs[marker_index][0][2]:.3f}]')
            
            # マーカー検出結果をパブリッシュ
            self.markers_publisher.publish(markers_msg)
            
            # デバッグ用の画像をパブリッシュ
            debug_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            debug_img_msg.header = msg.header
            self.debug_image_publisher.publish(debug_img_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetectorNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
