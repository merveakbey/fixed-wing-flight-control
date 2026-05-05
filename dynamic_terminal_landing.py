#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import threading # Terminali dondurmamak için eklendi
from pynput import keyboard

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data

class DynamicTerminalLanding(Node):
    def __init__(self):
        super().__init__('dynamic_terminal_landing_node')

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Görev Durum Değişkenleri
        self.is_takeoff_complete = False
        self.landing_requested = False  # L tuşuna basıldı mı?
        self.landing_locked = False     # Koordinatlar girilip rota kilitlendi mi?
        self.is_landed = False 

        # Devriye (Loiter) Noktası
        self.hold_x = 200.0
        self.hold_y = 200.0
        self.hold_z = 50.0

        # Varsayılan pist koordinatları (Terminalden güncellen40
        # ecek)
        self.runway_x = 0.0 
        self.runway_y = 0.0
        self.runway_yaw = 0.0 
         
        self.current_wp_index = 0
        self.alignment_waypoints = []

        # Klavye dinleyicisini başlat
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🚀 Sistem Başlatıldı. Otomatik kalkış ve devriye bekleniyor...")
        self.get_logger().info("💡 BİLGİ: İstediğiniz an 'L' tuşuna basarak terminalden iniş koordinatlarını girebilirsiniz!")

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)

    def set_arm(self, state):
        req = CommandBool.Request()
        req.value = state
        self.arm_client.call_async(req)

    def on_press(self, key):
        try:
            if key.char.lower() == 'l' and not self.landing_requested:
                self.landing_requested = True
                self.get_logger().info("⚠️ İNİŞ KOMUTU TETİKLENDİ! Lütfen terminale koordinatları girin (Uçuş devam ediyor...)")
                
                # Terminalden giriş alırken ROS'u dondurmamak için ayrı bir iş parçacığı (Thread) başlatıyoruz!
                input_thread = threading.Thread(target=self.get_terminal_input)
                input_thread.daemon = True
                input_thread.start()
        except AttributeError:
            pass

    def get_terminal_input(self):
        """Bu fonksiyon arka planda çalışır ve ana uçuş döngüsünü durdurmaz."""
        try:
            print("\n" + "="*40)
            print("🛬 DİNAMİK İNİŞ BİLGİSAYARI AKTİF")
            x_str = input("📍 İnecek hedefin X Koordinatı: ")
            y_str = input("📍 İnecek hedefin Y Koordinatı: ")
            yaw_str = input("🧭 İniş Yönü (Derece, örn 180): ")
            print("="*40 + "\n")

            self.runway_x = float(x_str)
            self.runway_y = float(y_str)
            self.runway_yaw = math.radians(float(yaw_str))

            # Girilen koordinatlara göre yeni bir görünmez kaydırak (Glide Slope) çiz
            self.generate_dynamic_approach(self.runway_x, self.runway_y, self.runway_yaw, 50.0, 600.0, 300.0)

            self.current_wp_index = 0
            self.landing_locked = True
            self.get_logger().info(f"🌐 Rota Kilitlendi! Uçak X={self.runway_x}, Y={self.runway_y} noktasına yöneliyor.")
        except ValueError:
            self.get_logger().error("❌ Hatalı giriş yaptınız! İniş iptal edildi. Uçak devriyeye devam ediyor.")
            self.landing_requested = False

    def generate_dynamic_approach(self, r_x, r_y, r_yaw, altitude, app_dist, base_dist):
        """Pist koordinatlarına göre ILS hizalanma noktalarını geriye doğru hesaplar"""
        final_x = r_x - (app_dist * math.cos(r_yaw))
        final_y = r_y - (app_dist * math.sin(r_yaw))
        
        base_yaw = r_yaw - (math.pi / 2.0)
        base_x = final_x - (base_dist * math.cos(base_yaw))
        base_y = final_y - (base_dist * math.sin(base_yaw))

        downwind_yaw = r_yaw - math.pi 
        downwind_x = base_x - (app_dist * math.cos(downwind_yaw))
        downwind_y = base_y - (app_dist * math.sin(downwind_yaw))

        self.alignment_waypoints = [
            {"name": "1. Ruzgar Alti", "x": downwind_x, "y": downwind_y, "z": altitude, "yaw": downwind_yaw},   
            {"name": "2. Donus Basi", "x": base_x, "y": base_y, "z": altitude, "yaw": downwind_yaw},
            {"name": "3. Son Yaklasma", "x": final_x, "y": final_y, "z": altitude, "yaw": base_yaw},
            {"name": "4. Piste Suzulus", "x": r_x, "y": r_y, "z": 0.0, "yaw": r_yaw} 
        ]

    def control_loop(self):
        if not self.current_state.connected: return

        # ==========================================
        # FAZ 1: KALKIŞ VE DEVRİYE (BEKLEME)
        # ==========================================
        if not self.landing_locked:
            pose = PoseStamped()
            pose.pose.position.x = self.hold_x
            pose.pose.position.y = self.hold_y
            pose.pose.position.z = self.hold_z
            self.local_pos_pub.publish(pose)

            if not self.is_takeoff_complete:
                if self.current_state.mode != "AUTO.TAKEOFF": 
                    self.set_mode("AUTO.TAKEOFF")
                elif not self.current_state.armed: 
                    self.set_arm(True)
                elif self.current_pose.pose.position.z > 40.0:
                    self.get_logger().info("✅ Kalkış tamam. Bekleme noktasına (Devriye) geçildi.")
                    self.is_takeoff_complete = True
            else:
                if self.current_state.mode != "OFFBOARD":
                    self.set_mode("OFFBOARD")
            
            # Koordinat girilmediyse kod burada kesilir ve turlamaya devam eder.
            return 

        # ==========================================
        # FAZ 2 & 3: DİNAMİK HİZALANMA VE PİSTE SÜZÜLÜŞ
        # ==========================================
        if self.current_wp_index < 4: 
            if self.current_state.mode != "OFFBOARD": 
                self.set_mode("OFFBOARD")
            
            target_wp = self.alignment_waypoints[self.current_wp_index]
            pose = PoseStamped()
            pose.pose.position.x = target_wp['x']
            pose.pose.position.y = target_wp['y']
            
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            
            if self.current_wp_index == 3:
                # Glide Slope Matematiği
                touchdown_x = self.runway_x - (40.0 * math.cos(self.runway_yaw))
                touchdown_y = self.runway_y - (40.0 * math.sin(self.runway_yaw))
                dist_to_touchdown = math.sqrt((touchdown_x - curr_x)**2 + (touchdown_y - curr_y)**2)
                
                ideal_z = (dist_to_touchdown / 600.0) * 50.0
                pose.pose.position.z = max(0.0, ideal_z) 

                if dist_to_touchdown <= 40.0:
                    self.get_logger().info("🛬 Teker koyma (Flare) aşamasına geçiliyor. Tutum (Attitude) devrede!")
                    self.current_wp_index = 4 
                    return
            else:
                pose.pose.position.z = target_wp['z']

            q = self.euler_to_quaternion(0.0, 0.0, target_wp['yaw'])
            pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
            self.local_pos_pub.publish(pose)

            # Noktaya varış kontrolü
            dist_2d = math.sqrt((curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2)
            if dist_2d < 40.0 and self.current_wp_index != 3: 
                self.get_logger().info(f"🔄 {target_wp['name']} aşıldı, sıradaki aşamaya geçiliyor.")
                self.current_wp_index += 1
            return

        # ==========================================
        # FAZ 4: İNİŞ, YUVARLANMA VE DURMA (ATTITUDE)
        # ==========================================
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_z = self.current_pose.pose.position.z
        
        att_msg = AttitudeTarget()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.type_mask = 7 
        
        dx = curr_x - self.runway_x
        dy = curr_y - self.runway_y

        y_error = dy * math.cos(self.runway_yaw) - dx * math.sin(self.runway_yaw)
        distance_to_stop_point = -(dx * math.cos(self.runway_yaw) + dy * math.sin(self.runway_yaw))

        target_yaw = self.runway_yaw - (y_error * 0.05) 
        target_roll = max(-0.139, min(0.139, y_error * 0.03)) if curr_z > 1.5 else 0.0 

        if distance_to_stop_point < 0.0:
            target_pitch = math.radians(-5.0) 
            target_thrust = 0.0               
            if curr_z < 0.5 and not self.is_landed:
                self.get_logger().info("🛑 DURMA NOKTASI GEÇİLDİ. MAKSİMUM FREN (DISARM).")
                self.set_arm(False)
                self.is_landed = True
        else:
            if curr_z > 2.0:
                target_pitch = math.radians(-1.0) 
                target_thrust = 0.0 
            elif curr_z > 0.2:
                target_pitch = math.radians(4.5) 
                target_thrust = 0.0 
            else:
                target_pitch = math.radians(-4.0) 
                target_thrust = 0.0 
                if not self.is_landed and distance_to_stop_point < 5.0:
                    self.get_logger().info(f"🎯 X={self.runway_x}, Y={self.runway_y} NOKTASINDA BAŞARIYLA DURDURULDU.")
                    self.set_arm(False)
                    self.is_landed = True   

        q_raw = self.euler_to_quaternion(target_roll, target_pitch, target_yaw)
        att_msg.orientation = Quaternion(x=float(q_raw[0]), y=float(q_raw[1]), z=float(q_raw[2]), w=float(q_raw[3]))
        att_msg.thrust = float(target_thrust)
        self.attitude_pub.publish(att_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTerminalLanding()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()