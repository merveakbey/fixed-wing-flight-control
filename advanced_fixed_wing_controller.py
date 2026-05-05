#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
from mavros_msgs.msg import State, AttitudeTarget
import math
import threading
import sys
import termios
import tty
import random
import time

class AdvancedFixedWingController(Node):
    def __init__(self):
        super().__init__('advanced_fixed_wing_controller')

        # --- PARAMETRELER (Devriye Noktası) ---
        self.declare_parameter('target_x', 100000.0) 
        self.declare_parameter('target_y', 200.0)   
        self.declare_parameter('target_z', 200.0)  
        
        # --- DURUM DEĞİŞKENLERİ ---
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Faz 1: Kalkış Değişkenleri
        self.is_takeoff_complete = False
        self.last_cmd_time = 0.0

        # Manevra Değişkenleri
        self.is_maneuvering = False
        self.maneuver_type = ""
        self.target_roll_rad = 0.0  
        self.crazy_roll_rate = 0.0
        self.crazy_pitch_rate = 0.0
        self.crazy_timer = None

        # İniş Değişkenleri
        self.landing_locked = False
        self.landing_requested = False
        self.is_landed = False 
        self.waiting_for_terminal_input = False
        self.current_wp_index = 0
        self.alignment_waypoints = []
        self.runway_x = 0.0 
        self.runway_y = 0.0
        self.runway_yaw = 0.0 

        # --- ROS 2 ABONELİKLER & YAYINCILAR ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   
            durability=DurabilityPolicy.VOLATILE,         
            history=HistoryPolicy.KEEP_LAST,              
            depth=1                                       
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.att_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        # --- ZAMANLAYICILAR ---
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz Ana Kontrol Döngüsü
        self.log_timer = self.create_timer(1.0, self.log_status)
        
        # --- KLAVYE DİNLEYİCİ İŞ PARÇACIĞI ---
        self.keyboard_thread = threading.Thread(target=self.wait_for_key)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.init_sequence()

    # ==========================================
    # YARDIMCI FONKSİYONLAR & CALLBACKLER
    # ==========================================
    def state_callback(self, msg): self.current_state = msg
    def pose_callback(self, msg): self.current_pose = msg

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def call_service(self, client, request):
        if client.wait_for_service(timeout_sec=3.0):
            client.call_async(request)

    def set_px4_param(self, param_id, integer_value=0):
        req = ParamSet.Request()
        req.param_id = param_id
        req.value.integer = integer_value
        self.param_client.call_async(req)

    def set_mode(self, mode):
        req = SetMode.Request(custom_mode=mode)
        self.mode_client.call_async(req)

    def set_arm(self, state):
        req = CommandBool.Request(value=state)
        self.arming_client.call_async(req)

    def log_status(self):
        if hasattr(self.current_pose, 'pose'):
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            z = self.current_pose.pose.position.z
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | İrtifa={z:.1f} | MOD: {self.current_state.mode}')

    # ==========================================
    # BAŞLANGIÇ & KLAVYE KONTROLÜ
    # ==========================================
    def init_sequence(self):
        self.get_logger().info('🚀 Sistem Başlatıldı... Otonom Kalkış döngüsü bekleniyor.')
        self.get_logger().info('💡 MANEVRALAR: R (Takla), L (Loop), I (Immelmann), K (Knife), C (Crazy Ivan)')
        self.get_logger().info('🛬 DİNAMİK İNİŞ: "p" tuşuna basarak terminalden iniş koordinatlarını girebilirsiniz!')
        
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        self.set_px4_param('COM_RC_IN_MODE', 1) 
        
        # Kalkış fazı `control_loop` içerisinde is_takeoff_complete ile kontrol edilecek

    def wait_for_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            while True:
                # Terminalden iniş koordinatı girilecekse ham okumayı dondur.
                if self.waiting_for_terminal_input:
                    time.sleep(0.1)
                    continue

                tty.setraw(fd)
                key = sys.stdin.read(1)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

                if not key:
                    continue

                z_alt = self.current_pose.pose.position.z if hasattr(self.current_pose, 'pose') else 0.0

                if key.lower() == 'r':
                    if z_alt > 150.0 and self.current_state.mode == "OFFBOARD":
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: TAKLA BAŞLIYOR !!!')
                        self.start_maneuver('roll', 5.0)
                    else:
                        self.get_logger().warn(f'GÜVENLİK UYARISI / MOD HATASI: İrtifa ({z_alt:.1f}m).')

                elif key.lower() == 'l':
                    if z_alt > 150.0 and self.current_state.mode == "OFFBOARD":
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: TAM ÇEMBER LOOP BAŞLIYOR !!!')
                        self.start_maneuver('loop', 4.0) 
                    else:
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa ({z_alt:.1f}m).')

                elif key.lower() == 'i':
                    if z_alt > 150.0 and self.current_state.mode == "OFFBOARD":
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: IMMELMANN DÖNÜŞÜ BAŞLIYOR !!!')
                        self.start_immelmann()
                    else:
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa ({z_alt:.1f}m).')

                elif key.lower() == 'k':
                    if z_alt > 150.0 and self.current_state.mode == "OFFBOARD":
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: KNIFE EDGE BAŞLIYOR !!!')
                        self.start_knife_edge()
                    else:
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa ({z_alt:.1f}m).')

                elif key.lower() == 'c':
                    if z_alt > 100.0 and self.current_state.mode == "OFFBOARD":
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: ÇILGIN İVAN KAÇIŞI BAŞLIYOR !!!')
                        self.start_crazy_ivan()
                    else:
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa ({z_alt:.1f}m).')

                elif key.lower() == 'p':
                    if not self.landing_requested:
                        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: DİNAMİK OTONOM İNİŞ (LANDING) EKRANI !!!')
                        self.landing_requested = True
                        self.waiting_for_terminal_input = True
                        
                        input_thread = threading.Thread(target=self.get_terminal_input)
                        input_thread.daemon = True
                        input_thread.start()

                elif key.lower() == 't':
                    self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: YENİDEN KALKIŞ (TAKEOFF) BAŞLIYOR !!!')
                    self.start_takeoff()

                elif key.lower() == 'q':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ==========================================
    # TERMİNALDEN İNİŞ BİLGİSİ ALMA (DİNAMİK İNİŞ)
    # ==========================================
    def get_terminal_input(self):
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

            if self.is_maneuvering:
                self.stop_maneuver()

            self.generate_dynamic_approach(self.runway_x, self.runway_y, self.runway_yaw, 50.0, 600.0, 300.0)
            self.current_wp_index = 0
            self.landing_locked = True
            self.is_landed = False
            self.get_logger().info(f"🌐 Rota Kilitlendi! Uçak X={self.runway_x}, Y={self.runway_y} noktasına yöneliyor.")
        except ValueError:
            self.get_logger().error("❌ Hatalı giriş yaptınız! İniş iptal edildi. Uçak devriyeye devam ediyor.")
            self.landing_requested = False
        finally:
            self.waiting_for_terminal_input = False 

    def generate_dynamic_approach(self, r_x, r_y, r_yaw, altitude, app_dist, base_dist):
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

    # ==========================================
    # AKROBASİ & MANEVRA METOTLARI
    # ==========================================
    def start_maneuver(self, m_type, duration):
        if self.is_maneuvering or self.landing_locked: return
        self.get_logger().info('OFFBOARD: Gövde Hızı (Body Rate) ile manevra başlatılıyor!')
        self.is_maneuvering = True
        self.maneuver_type = m_type
        self.timer_stop_maneuver = self.create_timer(duration, self.stop_maneuver)

    def stop_maneuver(self):
        self.get_logger().info('Manevra bitti, normal konum (Position) hedeflerine dönülüyor.')
        self.is_maneuvering = False
        self.maneuver_type = ""
        if hasattr(self, 'timer_stop_maneuver'): self.timer_stop_maneuver.cancel()
        self.target_roll_rad = 0.0

    def start_immelmann(self):
        if self.is_maneuvering or self.landing_locked: return
        self.get_logger().info('IMMELMANN FAZ 1: Yarım Loop (Tırmanış) başlıyor...')
        self.is_maneuvering = True
        self.maneuver_type = 'immelmann_pitch'
        self.timer_immelmann_phase2 = self.create_timer(1.9, self.immelmann_phase2)

    def immelmann_phase2(self):
        self.get_logger().info('IMMELMANN FAZ 2: Yarım Tono (Düzeltme) başlıyor...')
        self.maneuver_type = 'immelmann_roll'
        self.timer_immelmann_phase2.cancel()
        self.timer_immelmann_phase3 = self.create_timer(2.0, self.immelmann_phase3)

    def immelmann_phase3(self):
        self.get_logger().info('IMMELMANN FAZ 3: Düz Uçuşa Toparlanma...')
        self.maneuver_type = 'immelmann_roll2'
        self.timer_immelmann_phase3.cancel()
        self.timer_stop_maneuver = self.create_timer(3.9, self.stop_maneuver)

    def start_knife_edge(self):
        if self.is_maneuvering or self.landing_locked: return
        self.get_logger().info('KNIFE EDGE FAZ 1: 90 Derece Yatış Başlıyor...')
        self.is_maneuvering = True
        self.maneuver_type = 'knife_edge_enter'
        self.timer_ke_phase2 = self.create_timer(1.4, self.knife_edge_phase2)

    def knife_edge_phase2(self):
        self.get_logger().info('KNIFE EDGE FAZ 2: Bıçak Sırtında Tutunma...')
        self.maneuver_type = 'knife_edge_hold'
        self.timer_ke_phase2.cancel()
        self.timer_ke_phase3 = self.create_timer(3.0, self.knife_edge_phase3)

    def knife_edge_phase3(self):
        self.get_logger().info('KNIFE EDGE FAZ 3: Düz Uçuşa Toparlanma...')
        self.maneuver_type = 'knife_edge_exit'
        self.timer_ke_phase3.cancel()
        self.timer_stop_maneuver = self.create_timer(0.4, self.stop_maneuver)

    def start_crazy_ivan(self):
        if self.is_maneuvering or self.landing_locked: return
        self.get_logger().info('CRAZY IVAN: Tahmin edilemez manevralar devrede, tam gaz kaçış!')
        self.is_maneuvering = True
        self.maneuver_type = 'crazy_ivan'
        self.update_crazy_rates()
        self.crazy_timer = self.create_timer(1.5, self.update_crazy_rates)
        self.timer_stop_maneuver = self.create_timer(6.0, self.stop_crazy_ivan)

    def update_crazy_rates(self):
        self.crazy_roll_rate = random.uniform(-1.5, 1.5)
        self.crazy_pitch_rate = random.uniform(-1.0, 1.0) 

    def stop_crazy_ivan(self):
        if self.crazy_timer is not None: self.crazy_timer.cancel()
        self.stop_maneuver() 

    def start_takeoff(self):
        self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: YENİDEN KALKIŞ (TAKEOFF) BAŞLIYOR !!!')
        # Bütün kilitleri aç
        self.landing_locked = False
        self.landing_requested = False
        self.is_landed = False
        self.is_takeoff_complete = False 
        
        # Anında ilk komutları gönder ve spam sayacını sıfırla
        self.set_arm(True)
        self.set_mode('AUTO.TAKEOFF')
        self.last_cmd_time = time.time()

    # ==========================================
    # ANA KONTROL DÖNGÜSÜ (50 Hz)
    # ==========================================
    def control_loop(self):
        if not hasattr(self.current_pose, 'pose') or not self.current_state.connected:
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_z = self.current_pose.pose.position.z

        # ------------------------------------------
        # 1. DURUM: AKROBASİ MANEVRASI AKTİF (Type_mask: 128)
        # ------------------------------------------
        if self.is_maneuvering:
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.type_mask = 128 
            
            if self.maneuver_type == 'roll':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 1.7, 0.08, 0.2, 0.8
            elif self.maneuver_type == 'loop':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 0.0, -1.7, 0.0, 1.0
            elif self.maneuver_type == 'immelmann_pitch':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 0.0, -1.7, 0.0, 1.0
            elif self.maneuver_type == 'immelmann_roll':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 1.7, -1.7, 0.2, 1.0
            elif self.maneuver_type == 'immelmann_roll2':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 1.7, 0.0, 0.2, 1.0
            elif self.maneuver_type == 'knife_edge_enter':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 1.57, 0.0, 0.0, 1.0
            elif self.maneuver_type == 'knife_edge_hold':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = 0.0, 0.0, -0.4, 1.0
            elif self.maneuver_type == 'knife_edge_exit':
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust = -1.57, 0.0, 0.0, 0.8
            elif self.maneuver_type == 'crazy_ivan':
                msg.body_rate.x, msg.body_rate.y = self.crazy_roll_rate, self.crazy_pitch_rate
                msg.body_rate.z, msg.thrust = random.uniform(-0.5, 0.5), 1.0 
            
            self.att_pub.publish(msg)
            return

        # ------------------------------------------
        # 2. DURUM: DİNAMİK İNİŞ AKTİF (Koordinatlar girilmiş)
        # ------------------------------------------
        if self.landing_locked:
            if self.current_wp_index < 4: 
                if self.current_state.mode != "OFFBOARD": 
                    self.set_mode("OFFBOARD")
                
                target_wp = self.alignment_waypoints[self.current_wp_index]
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = target_wp['x']
                pose.pose.position.y = target_wp['y']
                
                if self.current_wp_index == 3: 
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
                self.pos_pub.publish(pose)

                dist_2d = math.sqrt((curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2)
                if dist_2d < 40.0 and self.current_wp_index != 3: 
                    self.get_logger().info(f"🔄 {target_wp['name']} aşıldı, sıradaki aşamaya geçiliyor.")
                    self.current_wp_index += 1
                return

            if self.current_wp_index >= 4:
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
                    target_pitch, target_thrust = math.radians(-5.0), 0.0               
                    if curr_z < 0.5 and not self.is_landed:
                        self.get_logger().info("🛑 DURMA NOKTASI GEÇİLDİ. MAKSİMUM FREN (DISARM).")
                        self.call_service(self.arming_client, CommandBool.Request(value=False))
                        self.is_landed = True
                else:
                    if curr_z > 2.0:
                        target_pitch, target_thrust = math.radians(-1.0), 0.0 
                    elif curr_z > 0.2:
                        target_pitch, target_thrust = math.radians(4.5), 0.0 
                    else:
                        target_pitch, target_thrust = math.radians(-4.0), 0.0 
                        if not self.is_landed and distance_to_stop_point < 5.0:
                            self.get_logger().info(f"🎯 X={self.runway_x}, Y={self.runway_y} NOKTASINDA BAŞARIYLA DURDURULDU.")
                            self.call_service(self.arming_client, CommandBool.Request(value=False))
                            self.is_landed = True   

                q_raw = self.euler_to_quaternion(target_roll, target_pitch, target_yaw)
                att_msg.orientation = Quaternion(x=float(q_raw[0]), y=float(q_raw[1]), z=float(q_raw[2]), w=float(q_raw[3]))
                att_msg.thrust = float(target_thrust)
                self.att_pub.publish(att_msg)
            return

        # ------------------------------------------
        # 3. DURUM: KALKIŞ VE DEVRİYE UÇUŞU (FAZ 1 - EKSİKSİZ)
        # ------------------------------------------
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.get_parameter('target_x').value
        msg.pose.position.y = self.get_parameter('target_y').value
        msg.pose.position.z = self.get_parameter('target_z').value
        self.pos_pub.publish(msg)

        # SPAM KORUMASI: Komutları yalnızca 1 saniye aralıklarla yolla
        current_time = time.time()
        
        if not self.is_takeoff_complete:
            if current_time - self.last_cmd_time > 1.0:
                if not self.current_state.armed: 
                    self.set_arm(True)
                elif self.current_state.mode != "AUTO.TAKEOFF": 
                    self.set_mode("AUTO.TAKEOFF")
                self.last_cmd_time = current_time

            if curr_z > 40.0:
                self.get_logger().info("✅ Kalkış tamam. Bekleme noktasına (Devriye) geçildi.")
                self.is_takeoff_complete = True
        else:
            if self.current_state.mode != "OFFBOARD":
                if current_time - self.last_cmd_time > 1.0:
                    self.set_mode("OFFBOARD")
                    self.last_cmd_time = current_time
def main(args=None):
    rclpy.init(args=args)
    node = AdvancedFixedWingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()