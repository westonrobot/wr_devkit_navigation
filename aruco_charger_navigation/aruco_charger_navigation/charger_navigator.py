#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Int32MultiArray
import math
import numpy as np
import time

class ChargerNavigatorNode(Node):
    def __init__(self):
        super().__init__('charger_navigator')
        
        # パラメータの宣言と取得
        self.declare_parameter('approach_distance', 0.5)    # マーカーからの目標距離（メートル）
        self.declare_parameter('linear_speed', 0.2)         # 直線速度（メートル/秒）
        self.declare_parameter('angular_speed', 0.3)        # 角速度（ラジアン/秒）
        self.declare_parameter('position_tolerance', 0.05)  # 位置許容誤差（メートル）
        self.declare_parameter('angle_tolerance', 0.1)      # 角度許容誤差（ラジアン）
        self.declare_parameter('docking_time', 5.0)         # 目標位置到達確認時間（秒）
        self.declare_parameter('direct_approach', True)     # 直接接近モード（マーカーを見つけたら直進優先）
        self.declare_parameter('control_rate', 5.0)         # 制御ループの周波数(Hz)
        self.declare_parameter('smoothing_factor', 0.7)     # コマンド平滑化係数(0-1)
        
        self.approach_distance = self.get_parameter('approach_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.docking_time = self.get_parameter('docking_time').value
        self.direct_approach = self.get_parameter('direct_approach').value
        self.control_rate = self.get_parameter('control_rate').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        
        # マーカーの相対位置のサブスクライバー
        self.marker_pose_subscription = self.create_subscription(
            PoseStamped,
            'marker_pose',
            self.marker_pose_callback,
            10)
            
        # 検出されたマーカーIDのサブスクライバー
        self.markers_subscription = self.create_subscription(
            Int32MultiArray,
            'aruco_markers',
            self.markers_callback,
            10)
        
        # ロボットの速度コマンドパブリッシャー
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)
            
        # ナビゲーション状態パブリッシャー
        self.status_publisher = self.create_publisher(
            String, 
            'navigation_status', 
            10)
            
        # ナビゲーション状態
        self.marker_detected = False
        self.current_marker_pose = None
        self.get_logger().info('充電ステーションナビゲーターを初期化しました')
        
        # 目標位置に到達した時間を記録
        self.docking_start_time = None
        self.is_docked = False
        
        # アプローチ動作を安定させるための変数
        self.stable_approach_count = 0
        self.last_angle = 0.0
        
        # 速度コマンドの平滑化のための変数
        self.last_cmd = Twist()
        self.command_sent_time = time.time()
        
        # 初期検出フラグ（最初の検出を特別扱い）
        self.first_detection = True
        self.first_detection_time = 0.0
        
        # ナビゲーションループのタイマー（周波数を下げる）
        timer_period = 1.0 / self.control_rate  # 5Hzなら0.2秒
        self.get_logger().info(f'ナビゲーションループ周期: {timer_period:.3f}秒 ({self.control_rate}Hz)')
        self.timer = self.create_timer(timer_period, self.navigation_callback)
        
    def markers_callback(self, msg):
        """
        検出されたマーカーIDを処理するコールバック
        """
        # マーカーが検出されたかどうかをセット
        was_detected = self.marker_detected
        self.marker_detected = len(msg.data) > 0
        
        # 新たにマーカーを検出した場合
        if self.marker_detected and not was_detected:
            # 初めての検出もしくは再検出
            if self.first_detection or (self.first_detection_time != 0.0 and time.time() - self.first_detection_time > 10.0): # 10秒以上経過で新規とみなす
                self.get_logger().info("マーカーを新たに検出しました - 安定化動作を開始")
                self.first_detection = True
                self.first_detection_time = time.time()
                
                # 新規検出時は動作を穏やかに
                self.last_cmd = Twist()
        
        if not self.marker_detected:
            # マーカーが見つからない場合、現在のマーカー姿勢をクリア
            self.current_marker_pose = None
            self.publish_status("SEARCHING")
            # マーカーが見つからなくなった場合、ドッキング時間もリセット
            self.docking_start_time = None
            self.is_docked = False
            # 安定カウントをリセット
            self.stable_approach_count = 0
        
    def marker_pose_callback(self, msg):
        """
        マーカーの姿勢を処理するコールバック
        """
        self.current_marker_pose = msg
        self.marker_detected = True
        
    def navigation_callback(self):
        """
        制御ループの主要ロジック（5Hzで実行）
        """
        # すでにドッキング完了している場合はスキップ
        if self.is_docked:
            return
            
        # 前回のコマンド送信からの経過時間を確認
        current_time = time.time()
        dt = current_time - self.command_sent_time
        
        # マーカーが検出されていない場合は回転して探索
        if not self.marker_detected or self.current_marker_pose is None:
            self.search_for_marker()
            # ドッキング時間をリセット
            self.docking_start_time = None
            # 安定カウントをリセット
            self.stable_approach_count = 0
            # コマンド送信時間を更新
            self.command_sent_time = current_time
            return
        
        # 現在のマーカーの位置
        marker_position = self.current_marker_pose.pose.position
        
        # マーカーまでの距離を計算
        distance = math.sqrt(marker_position.x**2 + marker_position.z**2)
        
        # マーカーまでの角度を計算（x-z平面での）- 座標系を変える場合はここを修正
        raw_angle = math.atan2(marker_position.x, marker_position.z)
        
        # デバッグ: マーカー位置と角度を表示
        self.get_logger().debug(f'マーカー位置: x={marker_position.x:.3f}, z={marker_position.z:.3f}, 角度={math.degrees(raw_angle):.1f}度')
        
        # 角度の変化率を計算して急激な動きを抑制（移動平均）
        angle_smoothed = 0.7 * raw_angle + 0.3 * self.last_angle
        self.last_angle = angle_smoothed
        
        # ナビゲーションステータスをログに出力
        self.get_logger().debug(f'マーカーまでの距離: {distance:.2f}m, 角度: {math.degrees(angle_smoothed):.1f}度')
        
        # 速度コマンドを初期化（前回の値をベースに）
        cmd = Twist()
        
        # 目標距離との差を計算
        distance_error = distance - self.approach_distance
        
        # 十分な距離と角度になったら停止
        if (abs(distance_error) < self.position_tolerance and
            abs(angle_smoothed) < self.angle_tolerance):
            # 目標位置に到達
            self.publish_status("DOCKED")
            
            # 初めて目標位置に到達した場合、時間を記録
            if self.docking_start_time is None:
                self.docking_start_time = time.time()
                self.get_logger().info('目標位置に到達しました。確認開始...')
            
            # 目標位置に到達してからの経過時間を確認
            elapsed_time = time.time() - self.docking_start_time
            
            # 設定時間以上目標位置に留まっている場合
            if elapsed_time >= self.docking_time:
                if not self.is_docked:
                    self.is_docked = True
                    self.get_logger().info(f'{self.docking_time}秒以上目標位置に留まりました。ナビゲーションを完了します。')
                    # システム終了を別スレッドで実行
                    self.create_timer(1.0, self.shutdown_system, oneshot=True)
            
            # 速度をゼロに設定し、前回の値との平滑化も行う
            cmd.linear.x = self.smoothing_factor * 0.0 + (1.0 - self.smoothing_factor) * self.last_cmd.linear.x
            cmd.angular.z = self.smoothing_factor * 0.0 + (1.0 - self.smoothing_factor) * self.last_cmd.angular.z
            
            self.cmd_vel_publisher.publish(cmd)
            # ナビゲーションコマンドをログ出力 - 値と状態を含める
            current_status = getattr(self, 'last_status', 'UNKNOWN')
            self.get_logger().info(f'NAV2コマンド: linear.x={cmd.linear.x:.3f} m/s, angular.z={cmd.angular.z:.3f} rad/s [{current_status}]')
            self.last_cmd = cmd
            self.command_sent_time = current_time
            return
        else:
            # 目標位置から外れた場合、タイマーをリセット
            self.docking_start_time = None
        
        # 新しい速度コマンドを計算（目標値）
        target_cmd = Twist()
        
        # 最初の検出から3秒間は特別な動作（前進優先）
        initial_approach_time = 3.0  # 秒
        if self.first_detection and self.first_detection_time != 0.0 and (time.time() - self.first_detection_time) < initial_approach_time:
            # 初期検出後の特別処理 - まっすぐ前進優先
            if abs(angle_smoothed) < 0.3:  # 約17度未満なら
                # 前進のみ
                target_cmd.linear.x = 0.3 * self.linear_speed
                # 最小限の角度調整
                target_cmd.angular.z = 0.2 * self.angular_speed * np.sign(angle_smoothed)
                self.publish_status("INITIAL_APPROACH")
                self.get_logger().info(f'初期アプローチ中: 前進優先 ({(time.time() - self.first_detection_time):.1f}/{initial_approach_time}秒)')
            else:
                # 角度が大きい場合は、非常に緩やかに調整
                target_cmd.angular.z = 0.3 * self.angular_speed * np.sign(angle_smoothed)
                self.publish_status("INITIAL_ALIGNING")
                self.get_logger().info(f'初期アプローチ中: 角度調整 ({math.degrees(angle_smoothed):.1f}度)')
            
            # 初期アプローチ時間が経過したらフラグをクリア
            if (time.time() - self.first_detection_time) >= initial_approach_time:
                self.first_detection = False
                self.get_logger().info("初期アプローチ完了 - 通常ナビゲーションへ移行")
        else:
            # 通常のアプローチ戦略
            if self.direct_approach:
                # 直接接近モード - 角度が極端でなければ、前進と回転を同時に行う
                large_angle_threshold = 0.4  # ラジアン（約23度）- より小さい値に
                
                if abs(angle_smoothed) > large_angle_threshold:
                    # 角度が大きい場合はまず向きを合わせる（低速で）
                    target_cmd.angular.z = self.angular_speed * 0.5 * np.sign(angle_smoothed)
                    self.publish_status("ALIGNING")
                    self.get_logger().debug(f'向きを調整中: {math.degrees(angle_smoothed):.1f}度')
                    # 安定カウントをリセット
                    self.stable_approach_count = 0
                else:
                    # 角度が適切な範囲なら、前進と角度調整を同時に行う
                    # 距離に応じた前進速度（遠いほど速く、近いほど遅く）
                    forward_speed = min(abs(distance_error), 0.3) / 0.3 * self.linear_speed
                    if distance_error > 0:
                        target_cmd.linear.x = forward_speed * 0.7  # 前進速度を70%に抑制
                        self.publish_status("APPROACHING")
                    else:
                        target_cmd.linear.x = -forward_speed * 0.3  # 後退は30%に抑制
                        self.publish_status("BACKING_UP")
                    
                    # 同時に角度も調整（前進中は角度調整を穏やかに）
                    angle_adjust_factor = 0.2 + 0.3 * (1.0 - min(forward_speed / self.linear_speed, 1.0))
                    target_cmd.angular.z = self.angular_speed * angle_adjust_factor * np.sign(angle_smoothed) * min(abs(angle_smoothed) / large_angle_threshold, 0.5)
                    
                    # 安定したアプローチの維持
                    self.stable_approach_count += 1
                    if self.stable_approach_count > 10:  # 安定していたら
                        # より穏やかに前進（急激な加速を避ける）
                        target_cmd.linear.x *= 1.1
            else:
                # 従来の方式 - 角度調整を優先（こちらも動作を穏やかに）
                if abs(angle_smoothed) > self.angle_tolerance * 2:  # 閾値を少し広げる
                    target_cmd.angular.z = self.angular_speed * 0.6 * np.sign(angle_smoothed)  # 60%に抑制
                    self.publish_status("ALIGNING")
                else:
                    # 距離に比例した速度（より低速）
                    speed_factor = min(abs(distance_error) / 0.4, 0.7)
                    target_cmd.linear.x = speed_factor * self.linear_speed * np.sign(distance_error)
                    
                    # 小さな角度補正も同時に行う
                    target_cmd.angular.z = 0.3 * self.angular_speed * np.sign(angle_smoothed) * min(abs(angle_smoothed) / 0.1, 0.6)
                    
                    if distance_error > 0:
                        self.publish_status("APPROACHING")
                    else:
                        self.publish_status("BACKING_UP")
        
        # 前回の速度コマンドと新しい目標値を滑らかに補間
        cmd.linear.x = self.smoothing_factor * target_cmd.linear.x + (1.0 - self.smoothing_factor) * self.last_cmd.linear.x
        cmd.angular.z = self.smoothing_factor * target_cmd.angular.z + (1.0 - self.smoothing_factor) * self.last_cmd.angular.z
        
        # 速度コマンドをパブリッシュ
        self.cmd_vel_publisher.publish(cmd)
        # ナビゲーションコマンドをログ出力 - 値と状態を含める
        current_status = getattr(self, 'last_status', 'UNKNOWN')
        self.get_logger().info(f'NAV2コマンド: linear.x={cmd.linear.x:.3f} m/s, angular.z={cmd.angular.z:.3f} rad/s [{current_status}]')
        
        # 今回のコマンドと時間を保存
        self.last_cmd = cmd
        self.command_sent_time = current_time
        
    def search_for_marker(self):
        """
        マーカーを探すために回転する
        """
        # 前回の速度と滑らかに補間した回転コマンドを送信
        cmd = Twist()
        target_angular_z = self.angular_speed * 0.3  # 探索時の回転速度を30%に低減
        cmd.angular.z = self.smoothing_factor * target_angular_z + (1.0 - self.smoothing_factor) * self.last_cmd.angular.z
        self.cmd_vel_publisher.publish(cmd)
        # 探索コマンドをログ出力
        self.get_logger().info(f'NAV2探索コマンド: linear.x={cmd.linear.x:.3f} m/s, angular.z={cmd.angular.z:.3f} rad/s [SEARCHING]')
        self.last_cmd = cmd
        self.publish_status("SEARCHING")
        
    def publish_status(self, status):
        """
        ナビゲーション状態をパブリッシュ
        """
        # 前回と状態が変わった場合のみログ出力
        if not hasattr(self, 'last_status') or status != self.last_status:
            self.get_logger().info(f'ナビゲーション状態: {status}')
            self.last_status = status
    
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
    
    def shutdown_system(self):
        """
        システムをシャットダウンする
        """
        # 最終的に速度をゼロにして安全に停止
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        # 停止コマンドをログ出力
        self.get_logger().info(f'NAV2停止コマンド: linear.x=0.000 m/s, angular.z=0.000 rad/s [SHUTDOWN]')

        self.get_logger().info('ナビゲーションを正常に完了しました。システムを終了します。')
        
        # ROSノードを終了
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = ChargerNavigatorNode()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時に確実に停止コマンドを送信
        stop_cmd = Twist()
        navigator.cmd_vel_publisher.publish(stop_cmd)
        navigator.get_logger().info('終了時NAV2停止コマンド: linear.x=0.000 m/s, angular.z=0.000 rad/s [TERMINATED]')
        navigator.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
