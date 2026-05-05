import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
import math
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust
import threading
import sys
import termios
import tty
import random

class FixedWingPositionControl(Node):
    def __init__(self):
        super().__init__('fixed_wing_controller')

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 200.0)  
        
        self.current_pose = None
        self.current_mode = "BILINMIYOR"

        self.is_maneuvering = False
        self.maneuver_type = ""
        self.target_roll_rad = 0.0  

        self.current_quat = None
        self.locked_yaw = 0.0
        self.locked_pitch = 0.0


        self.crazy_roll_rate = 0.0
        self.crazy_pitch_rate = 0.0
        self.crazy_timer = None
        
        self.timer = self.create_timer(0.02, self.publish_control_commands)
        
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_callback, 
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   
            durability=DurabilityPolicy.VOLATILE,         
            history=HistoryPolicy.KEEP_LAST,              
            depth=1                                       
        )

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        self.att_pub = self.create_publisher(
            AttitudeTarget, 
            '/mavros/setpoint_raw/attitude', 
            10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )

        self.log_timer = self.create_timer(1.0, self.log_position)
        
        self.keyboard_thread = threading.Thread(target=self.wait_for_key)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.init_sequence()

    def wait_for_key(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
                
                if key.lower() == 'r':
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: İRTİFA YETERLİ, TAKLA BAŞLIYOR !!!')
                            self.start_maneuver('roll', 5.0)
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m). Takla komutu YALITILDI.')

                elif key.lower() == 'l':
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: TAM ÇEMBER LOOP BAŞLIYOR !!!')
                            self.start_maneuver('loop', 4.0) 
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m). Loop komutu YALITILDI.')

                elif key.lower() == 'i':
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: IMMELMANN DÖNÜŞÜ BAŞLIYOR !!!')
                            self.start_immelmann()
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m).')

                elif key.lower() == 'k':
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: KNIFE EDGE BAŞLIYOR !!!')
                            self.start_knife_edge()
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m).')


                elif key.lower() == 'c':
                    if self.current_pose is not None and self.current_pose.z > 100.0:
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: ÇILGIN İVAN KAÇIŞI BAŞLIYOR !!!')
                            self.start_crazy_ivan()
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa çok düşük! (Mevcut: {mevcut_irtifa:.1f}m). Çarparsın!')


                elif key.lower() == 'p':
                    self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: OTONOM İNİŞ (LANDING) BAŞLIYOR !!!')
                    self.start_landing()


                elif key.lower() == 't':
                    self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: YENİDEN KALKIŞ (TAKEOFF) BAŞLIYOR !!!')
                    self.start_takeoff()

                if key.lower() == 'q':
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def init_sequence(self):
        self.get_logger().info('Konum Kontrolu baslatiliyor... TAKLA İÇİN "K" TUŞUNA BASIN (İrtifa > 150m ise çalışır).')
        
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        self.set_px4_param('COM_RC_IN_MODE', 1) 
        
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        self.timer_offboard = self.create_timer(10.0, self.switch_to_offboard)

    def set_px4_param(self, param_id, integer_value=0):
        req = ParamSet.Request()
        req.param_id = param_id
        req.value.integer = integer_value
        self.param_client.call_async(req)

    def switch_to_offboard(self):
        self.get_logger().info('OFFBOARD moduna geçiliyor, hedefe gidiliyor... (Takla atmak için terminalden "k" tuşuna basabilirsiniz)')
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)
        self.timer_offboard.cancel()

    def start_maneuver(self, m_type, duration):
        if self.is_maneuvering:
            return
            
        self.get_logger().info('OFFBOARD: Gövde Hızı (Body Rate) ile saf takla başlatılıyor!')
        self.is_maneuvering = True
        self.maneuver_type = m_type
        
        self.timer_stop_maneuver = self.create_timer(duration, self.stop_maneuver)

    def stop_maneuver(self):
        self.get_logger().info('Takla bitti, normal konum (Position) hedeflerine dönülüyor.')
        self.is_maneuvering = False
        self.maneuver_type = ""
        self.timer_stop_maneuver.cancel()
        
        self.target_roll_rad = 0.0

    def start_immelmann(self):
        if self.is_maneuvering:
            return
        
        self.get_logger().info('IMMELMANN FAZ 1: Yarım Loop (Tırmanış) başlıyor...')
        self.is_maneuvering = True
        self.maneuver_type = 'immelmann_pitch'
        
        self.timer_immelmann_phase2 = self.create_timer(1.9, self.immelmann_phase2)

    def immelmann_phase2(self):
        self.get_logger().info('IMMELMANN FAZ 2: Yarım Tono (Düzeltme) başlıyor...')
        self.maneuver_type = 'immelmann_roll'
        
        self.timer_immelmann_phase2.cancel()
        
        self.timer_immelmann_phase3 = self.create_timer(2.0, self.immelmann_phase3)
    #    self.timer_stop_maneuver = self.create_timer(2.0, self.stop_maneuver)
    # immelmann hareketinden sonra ekstra roll atmasını kapamak istersem yukarıdakini açacağım

    def immelmann_phase3(self):
        self.get_logger().info('IMMELMANN FAZ 3: Yarım Tono (Düzeltme) başlıyor...')
        self.maneuver_type = 'immelmann_roll2'
        
        self.timer_immelmann_phase3.cancel()
        
        self.timer_stop_maneuver = self.create_timer(3.9, self.stop_maneuver)


    def start_knife_edge(self):
        if self.is_maneuvering:
            return
        
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
        if self.is_maneuvering:
            return
        
        self.get_logger().info('CRAZY IVAN: Tahmin edilemez manevralar devrede, tam gaz kaçış!')
        self.is_maneuvering = True
        self.maneuver_type = 'crazy_ivan'
        
        # İlk rastgele değerleri hemen ata
        self.update_crazy_rates()
        
        # ZEKASI BURADA: Her 1.5 saniyede bir uçağın yönünü ve açısını rastgele değiştir!
        self.crazy_timer = self.create_timer(1.5, self.update_crazy_rates)
        
        # 10 saniye boyunca havada çırpınsın, sonra normal uçuştan devam etsin
        self.timer_stop_maneuver = self.create_timer(6.0, self.stop_crazy_ivan)

    def update_crazy_rates(self):
        # -1.5 ile +1.5 radyan arası agresif sağ/sol yatış
        self.crazy_roll_rate = random.uniform(-1.5, 1.5)
        # Stall olmamak için burun (pitch) hareketini -1.0 ile +1.0 arası sınırlıyoruz
        self.crazy_pitch_rate = random.uniform(-1.0, 1.0) 
        self.get_logger().info(f'>> Crazy Ivan Güncelleme: Roll: {self.crazy_roll_rate:.2f}, Pitch: {self.crazy_pitch_rate:.2f}')

    def stop_crazy_ivan(self):
        # Çılgınlık bittiğinde rastgele sayı üreten zamanlayıcıyı kapat
        if self.crazy_timer is not None:
            self.crazy_timer.cancel()
        self.stop_maneuver() # Uçağı tekrar düz uçuşa al


    def start_landing(self):
        self.get_logger().info('Otopilota İniş (AUTO.LAND) komutu gönderiliyor, süzülme rotasına giriliyor...')
        
        # Eğer uçak o an takla atıyorsa veya kaçış manevrasındaysa önce onu durduralım
        if self.is_maneuvering:
            self.stop_maneuver()
            
        # Modu AUTO.LAND olarak değiştiriyoruz
        req = SetMode.Request(custom_mode='AUTO.LAND')
        self.mode_client.call_async(req)

    def start_takeoff(self):
        self.get_logger().info('Motorlar yeniden Arm ediliyor ve Kalkış moduna geçiliyor...')
        
        # 1. Uçağı tekrar Arm et (İndiğinde otomatik Disarm olmuştu)
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        
        # 2. Modu AUTO.TAKEOFF olarak ayarla
        req = SetMode.Request(custom_mode='AUTO.TAKEOFF')
        self.mode_client.call_async(req)
        
        # 3. Offboard'a geçiş sayacını iptal edip 10 saniyeden baştan başlatıyoruz
        if hasattr(self, 'timer_offboard') and self.timer_offboard is not None:
            self.timer_offboard.cancel()
            
        self.timer_offboard = self.create_timer(10.0, self.switch_to_offboard)

    def publish_control_commands(self):
        if self.is_maneuvering:
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.type_mask = 128 
            
            if self.maneuver_type == 'roll':
                msg.body_rate.x = 1.7  
                msg.body_rate.y = 0.08  
                msg.body_rate.z = 0.2  
                msg.thrust = 0.8        
                
            elif self.maneuver_type == 'loop':
                msg.body_rate.x = 0.0
                msg.body_rate.y = -1.7 
                msg.body_rate.z = 0.0
                msg.thrust = 1.0        

            elif self.maneuver_type == 'immelmann_pitch':
                msg.body_rate.x = 0.0
                msg.body_rate.y = -1.7  
                msg.body_rate.z = 0.0
                msg.thrust = 1.0        
                
            elif self.maneuver_type == 'immelmann_roll':
                msg.body_rate.x = 1.7   
                msg.body_rate.y = -1.7   
                msg.body_rate.z = 0.2
                msg.thrust = 1.0        

            elif self.maneuver_type == 'immelmann_roll2':
                msg.body_rate.x = 1.7   
                msg.body_rate.y = 0.0  
                msg.body_rate.z = 0.2
                msg.thrust = 1.0        
            
            elif self.maneuver_type == 'knife_edge_enter':
                msg.body_rate.x = 1.57   
                msg.body_rate.y = 0.0  
                msg.body_rate.z = 0.0  
                msg.thrust = 1.0        

            elif self.maneuver_type == 'knife_edge_hold':
                msg.body_rate.x = 0.0   
                msg.body_rate.y = 0.0   
                msg.body_rate.z = -0.4 
                msg.thrust = 1.0        

            elif self.maneuver_type == 'knife_edge_exit':
                msg.body_rate.x = -1.57 
                msg.body_rate.y = 0.0  
                msg.body_rate.z = 0.0  
                msg.thrust = 0.8       

            elif self.maneuver_type == 'crazy_ivan':
                msg.body_rate.x = self.crazy_roll_rate   
                msg.body_rate.y = self.crazy_pitch_rate  
                # Uçağın kuyruğunu da (Yaw) hafifçe rastgele savur ki hedef şaşırsın
                msg.body_rate.z = random.uniform(-0.5, 0.5) 
                msg.thrust = 3.40 # Kaçış manevrası olduğu için tam gaz ivmelen
            
            self.att_pub.publish(msg)
            
        else:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = self.get_parameter('target_x').value
            msg.pose.position.y = self.get_parameter('target_y').value
            msg.pose.position.z = self.get_parameter('target_z').value
            
            self.pos_pub.publish(msg)

    
    def pose_callback(self, msg):
        self.current_pose = msg.pose.position
        self.current_quat = msg.pose.orientation

    def state_callback(self, msg):
        self.current_mode = msg.mode
 
    def log_position(self):
        if self.current_pose is not None:
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | İrtifa={z:.1f} | MOD: {self.current_mode}')

    def call_service(self, client, request):
        if client.wait_for_service(timeout_sec=3.0):
            client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()