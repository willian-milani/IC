#!/usr/bin/env python3
"""Navegacao unificada GPS + LiDAR para Husky (ROS 2 Jazzy / Gazebo) - v5.

- Um unico publicador em /platform/cmd_vel (TwistStamped).
- 10 Goals GPS selecionados dos 37 pontos do percurso anterior.
- Raio de chegada configuravel (padrao 1,0 m).
- LiDAR PointCloud2 processado no proprio no: nao exige detector/analisador externos.
- GPS normal -> AVOID_TURN -> AVOID_FORWARD -> GPS normal.
- Tempo de desvio e confirmacao de passagem usam o relogio da ODOMETRIA
  (tempo de simulacao no Gazebo); watchdog adicional usa tempo real.
- Durante AVOID_FORWARD, registra a distancia percorrida por odometria,
  verifica o setor de retorno ao Goal e exige confirmacao temporal.
- Falta de leitura, manobra sem corredor valido ou limite de tentativas -> STOP/BLOCKED.
- Parada com Ctrl+C: publica 30 comandos nulos antes de encerrar.

ATENCAO: os limites geometricos abaixo sao PROVISORIOS. Confirmar tamanho do
Husky no SDF, campo de visao do LiDAR e altura dos obstaculos antes de rodar
sem supervisao. Este e um controlador experimental, nao um planejador local
com garantia geometrica completa.
"""

import math
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2


# Subconjunto EXATO de pontos do percurso original (indices 1-based):
# 1, 5, 9, 13, 17, 21, 25, 29, 33, 37.
# Inicio/apice/fim das 3 curvas em U, e ultimo ponto do caminho 4.
GOALS_GPS = [
    (-15.00010790337, -46.99980753637),  # 01 - Entrada da curva 1
    (-15.00007251889, -46.99977679985),  # 02 - Apice da curva 1
    (-15.00003713441, -46.99980753637),  # 03 - Saida da curva 1
    (-15.00003603847, -47.00019246363),  # 04 - Entrada da curva 2
    (-15.00000065400, -47.00022320015),  # 05 - Apice da curva 2
    (-14.99996526952, -47.00019246363),  # 06 - Saida da curva 2
    (-14.99996526952, -46.99980753637),  # 07 - Entrada da curva 3
    (-14.99992878910, -46.99977679985),  # 08 - Apice da curva 3
    (-14.99989340462, -46.99980753637),  # 09 - Saida da curva 3
    (-14.99989285665, -47.00018600013),  # 10 - Destino final
]


class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_node')

        defaults = {
            'gps_topic': '/sensors/gps/navsat',
            'odom_topic': '/platform/odom',
            'cloud_topic': '/sensors/seyond_lidar/scan/points',
            'cmd_vel_topic': '/platform/cmd_vel',
            'goal_radius': 1.0,
            'enable_avoidance': True,
            # GPS: mesmos ganhos e limites do controlador validado.
            'k_linear': 0.30,
            'k_angular': 1.50,
            'max_linear_speed': 0.20,
            'max_angular_speed': 0.45,
            # Geometria CONSERVADORA PROVISORIA em coordenadas base_link.
            'lidar_offset_x': 0.25,           # TF base_link -> seyond_lidar_laser
            'robot_half_width': 0.55,         # meia largura + margem (validar no SDF)
            'rotation_clearance_radius': 0.82, # raio de varredura ao girar (validar)
            'min_z': -0.40,                   # filtro relativo ao LiDAR
            'max_z': 0.30,
            'lidar_max_range': 3.0,
            'min_cloud_points': 30,           # pontos validos no pacote bruto
            # Limiares de X em relacao ao centro de base_link.
            'obstacle_trigger_distance': 1.45,
            'emergency_stop_distance': 0.85,
            'avoid_forward_stop_distance': 1.10,
            'avoid_candidate_min_clearance': 1.45,
            'avoid_release_distance': 1.90,
            'avoid_min_travel': 1.20,
            'avoid_speed': 0.08,
            'avoid_turn_speed': 0.22,
            'avoid_turn_tolerance_deg': 4.0,
            'avoid_hold_seconds': 1.5,
            'max_avoid_seconds': 40.0,  # segundos de SIMULACAO (stamp de /platform/odom)
            'max_avoid_wall_seconds': 240.0,  # watchdog independente de simulacao
            'max_avoid_attempts': 3,
            # Leituras de 12-13 Hz: tempo para tolerar lacunas curtas.
            'lidar_timeout': 1.5,  # tolera lacunas observadas de ate 0.66 s
            'gps_timeout': 3.0,  # GPS ~0.94 Hz; lacunas observadas de ate 1.355 s
            'odom_timeout': 1.5,
            'log_period': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.goals = list(GOALS_GPS)
        self.goal_index = 0
        self.finished = False
        self.state = 'GPS_NAVIGATION'
        self.stopping = False

        self.latitude = None
        self.longitude = None
        self.origin_latitude = None
        self.origin_longitude = None
        self.x = None
        self.y = None
        self.yaw = None
        self.gps_received_at = None
        self.odom_received_at = None
        self.lidar_received_at = None
        self.lidar_frame_warning = False
        self.cloud_points_xy = None
        self.cloud_valid = False
        self.lidar_issue = 'aguardando primeira mensagem'
        self.gps_issue = 'aguardando GPS'
        self.gps_messages = 0
        self.lidar_messages = 0
        self.odom_messages = 0
        self.last_heartbeat_at = 0.0
        self.odom_issue = 'aguardando odometria'

        self.avoid_target_yaw = None
        self.avoid_start_xy = None
        self.sim_time_sec = None       # tempo do header de /platform/odom
        self.avoid_started_sim = None  # inicio do ciclo em tempo de simulacao
        self.avoid_started_wall = None # watchdog em tempo real
        self.avoid_last_odom_xy = None
        self.avoid_traveled_odom = 0.0
        self.avoid_attempts = 0
        self.avoid_clear_since = None
        self.avoid_selected_angle = None
        self.last_log_at = 0.0
        self.last_warning_at = 0.0

        self._validate_configuration()

        # Referencias explicitas e contadores para diagnostico.
        # GPS/LiDAR: QoS BEST_EFFORT aceita publicadores BEST_EFFORT ou RELIABLE.
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.lidar_sub = None
        if self.enable_avoidance:
            self.lidar_sub = self.create_subscription(
                PointCloud2, self.cloud_topic, self.lidar_callback,
                qos_profile_sensor_data
            )
        self.cmd_pub = self.create_publisher(
            TwistStamped, self.cmd_vel_topic, 10
        )
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Navegacao UNIFICADA iniciada | {len(self.goals)} Goals | '
            f'raio={self.goal_radius:.2f} m | '
            f'Avoidance={"ATIVADO" if self.enable_avoidance else "DESATIVADO"}'
        )
        self.get_logger().info(
            f'GPS={self.gps_topic} | LiDAR={self.cloud_topic} | '
            f'Odometry={self.odom_topic} | CMD={self.cmd_vel_topic}'
        )
        self.get_logger().warn(
            'VALIDAR no SDF: robot_half_width, rotation_clearance_radius, '
            'campo de visao e filtro de altura do LiDAR.'
        )
        self.get_logger().info(
            'V5: tempo de desvio baseado no stamp de /platform/odom (Gazebo); '
            'tempo real usado apenas para watchdog. '
            f'Limites={self.max_avoid_seconds:.0f}s simulacao / '
            f'{self.max_avoid_wall_seconds:.0f}s reais.'
        )

    def _validate_configuration(self):
        positive = (
            'goal_radius', 'max_linear_speed', 'max_angular_speed',
            'robot_half_width', 'rotation_clearance_radius',
            'obstacle_trigger_distance', 'emergency_stop_distance',
            'avoid_forward_stop_distance', 'avoid_candidate_min_clearance',
            'avoid_release_distance', 'avoid_min_travel',
            'avoid_speed', 'avoid_turn_speed', 'max_avoid_seconds',
            'max_avoid_wall_seconds',
            'lidar_timeout', 'gps_timeout', 'odom_timeout',
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'Parametro {name} deve ser > 0 e finito.')
        if self.avoid_release_distance <= self.obstacle_trigger_distance:
            raise ValueError('avoid_release_distance deve superar obstacle_trigger_distance.')
        if self.min_z >= self.max_z:
            raise ValueError('min_z deve ser menor que max_z.')
        if int(self.max_avoid_attempts) < 1:
            raise ValueError('max_avoid_attempts deve ser >= 1.')

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def warn_throttled(self, text, period=2.0):
        now = time.monotonic()
        if now - self.last_warning_at >= period:
            self.get_logger().warn(text)
            self.last_warning_at = now

    def gps_callback(self, msg):
        self.gps_messages += 1
        if self.gps_messages == 1:
            self.get_logger().info('Primeira mensagem GPS recebida no controlador.')
        if (msg.status.status < 0 or not math.isfinite(msg.latitude)
                or not math.isfinite(msg.longitude)):
            self.latitude = self.longitude = None
            self.gps_received_at = None
            self.gps_issue = f'fix invalido status={msg.status.status}'
            self.warn_throttled('GPS sem fix valido; aguardando dados.')
            return
        self.latitude, self.longitude = msg.latitude, msg.longitude
        self.gps_issue = 'OK'
        self.gps_received_at = time.monotonic()
        if self.origin_latitude is None:
            self.origin_latitude, self.origin_longitude = self.latitude, self.longitude
            self.get_logger().info(
                f'Origem GPS: {self.origin_latitude:.10f}, '
                f'{self.origin_longitude:.10f}'
            )

    def odom_callback(self, msg):
        self.odom_messages += 1
        new_x = float(msg.pose.pose.position.x)
        new_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        new_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if not all(math.isfinite(v) for v in (new_x, new_y, new_yaw)):
            self.odom_issue = 'pose de odometria nao finita'
            self.odom_received_at = None
            return
        if stamp_sec <= 0.0 or not math.isfinite(stamp_sec):
            self.odom_issue = 'timestamp de simulacao ausente na odometria'
            self.sim_time_sec = None
            self.odom_received_at = None
            return

        if (self.sim_time_sec is not None and
                stamp_sec < self.sim_time_sec - 0.05 and
                self.state in ('AVOID_TURN', 'AVOID_FORWARD')):
            self.set_blocked('Relogio de simulacao retrocedeu durante desvio.')

        self.x, self.y, self.yaw = new_x, new_y, new_yaw
        self.sim_time_sec = stamp_sec
        self.odom_received_at = time.monotonic()
        self.odom_issue = 'OK'

        if self.state == 'AVOID_FORWARD':
            if self.avoid_last_odom_xy is not None:
                dx = new_x - self.avoid_last_odom_xy[0]
                dy = new_y - self.avoid_last_odom_xy[1]
                step = math.hypot(dx, dy)
                if step > 0.30:
                    self.set_blocked(
                        f'Salto de odometria durante passagem: {step:.2f}m.'
                    )
                    return
                self.avoid_traveled_odom += step
            self.avoid_last_odom_xy = (new_x, new_y)

    def lidar_callback(self, msg):
        self.lidar_messages += 1
        self.lidar_received_at = time.monotonic()
        if self.lidar_messages == 1:
            self.get_logger().info(
                f'Primeira mensagem LiDAR recebida | frame={msg.header.frame_id!r} | '
                f'total={msg.width * msg.height} pontos.'
            )
        # As regioes do codigo usam a orientacao do seyond_lidar_laser.
        # TF verificada anteriormente: offset [0.25, 0, 0.492], RPY=0.
        if msg.header.frame_id != 'seyond_lidar_laser':
            self.cloud_valid = False
            self.lidar_received_at = time.monotonic()
            self.lidar_issue = f'frame inesperado: {msg.header.frame_id!r}'
            self.warn_throttled(
                f'Frame inesperado {msg.header.frame_id!r}; parando por seguranca.'
            )
            return

        try:
            raw = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True
            )
            points = np.asarray(raw, dtype=np.float32).reshape(-1, 3)
            finite = np.isfinite(points).all(axis=1)
            points = points[finite]
            if points.shape[0] < int(self.min_cloud_points):
                self.lidar_issue = (f'poucos pontos na nuvem: {points.shape[0]} '
                                    f'(minimo={self.min_cloud_points})')
                self.cloud_valid = False
                self.lidar_received_at = time.monotonic()
                return
            height_mask = ((points[:, 2] >= self.min_z)
                           & (points[:, 2] <= self.max_z))
            relevant = points[height_mask]
            if relevant.size:
                # Transformacao para base_link: translacao +0.25 m em X,
                # rotacao nula confirmada por tf2_echo.
                xy = relevant[:, :2].copy()
                xy[:, 0] += self.lidar_offset_x
                self.cloud_points_xy = xy
            else:
                self.cloud_points_xy = np.empty((0, 2), dtype=np.float32)
            self.cloud_valid = True
            self.lidar_issue = 'OK'
            self.lidar_received_at = time.monotonic()
        except Exception as exc:
            self.cloud_valid = False
            self.lidar_received_at = time.monotonic()
            self.lidar_issue = f'erro no processamento: {str(exc)[:130]}'
            self.warn_throttled(f'Erro ao processar PointCloud2: {exc}')

    def fresh(self, last, timeout):
        return last is not None and time.monotonic() - last <= timeout

    def sensor_diagnostics(self):
        """Mostra qual entrada impediu o movimento, sem desabilitar a seguranca."""
        now = time.monotonic()

        def status(label, stamp, timeout, present, issue):
            age = None if stamp is None else now - stamp
            if not present:
                return f'{label}: INVALIDO ({issue})'
            if stamp is None:
                return f'{label}: SEM MENSAGEM'
            if age > timeout:
                return f'{label}: DESATUALIZADO ({age:.2f}s > {timeout:.2f}s)'
            return f'{label}: OK ({age:.2f}s)'

        gps_ok = (self.latitude is not None and self.longitude is not None
                  and self.origin_latitude is not None)
        odom_ok = (self.x is not None and self.y is not None and self.yaw is not None)
        result = [status('GPS', self.gps_received_at, self.gps_timeout,
                         gps_ok, self.gps_issue),
                  status('ODOM', self.odom_received_at, self.odom_timeout,
                         odom_ok, self.odom_issue)]
        if self.enable_avoidance:
            result.append(status('LiDAR', self.lidar_received_at,
                                 self.lidar_timeout, self.cloud_valid,
                                 self.lidar_issue))
        return ' | '.join(result)

    def sensor_heartbeat(self):
        """Conta callbacks recebidos, separando recepcao de validade."""
        now = time.monotonic()
        if now - self.last_heartbeat_at < 5.0:
            return
        self.last_heartbeat_at = now
        self.get_logger().info(
            'HEARTBEAT | callbacks GPS={} ODOM={} LiDAR={} | {}'.format(
                self.gps_messages, self.odom_messages, self.lidar_messages,
                self.sensor_diagnostics()
            )
        )

    def inputs_ready(self):
        if (self.latitude is None or self.longitude is None or
                self.yaw is None or self.x is None or self.y is None or
                self.origin_latitude is None):
            return False
        if not self.fresh(self.gps_received_at, self.gps_timeout):
            return False
        if (not self.fresh(self.odom_received_at, self.odom_timeout)
                or self.sim_time_sec is None):
            return False
        if self.enable_avoidance:
            if not self.cloud_valid or not self.fresh(
                self.lidar_received_at, self.lidar_timeout
            ):
                return False
        return True

    def gps_to_local(self, lat, lon):
        earth_radius = 6378137.0
        lat0 = math.radians(self.origin_latitude)
        dx = math.radians(lon - self.origin_longitude)
        dy = math.radians(lat - self.origin_latitude)
        return (dx * earth_radius * math.cos(lat0), dy * earth_radius)

    def goal_error(self):
        goal_lat, goal_lon = self.goals[self.goal_index]
        current_x, current_y = self.gps_to_local(self.latitude, self.longitude)
        target_x, target_y = self.gps_to_local(goal_lat, goal_lon)
        dx, dy = target_x - current_x, target_y - current_y
        return math.hypot(dx, dy), self.wrap(math.atan2(dy, dx) - self.yaw)

    def advance_goals(self):
        if self.finished:
            return
        while self.goal_index < len(self.goals):
            distance, _ = self.goal_error()
            if distance > self.goal_radius:
                break
            self.get_logger().info(
                f'GOAL {self.goal_index + 1}/{len(self.goals)} ALCANCADO! '
                f'dist={distance:.2f} m, raio={self.goal_radius:.2f} m'
            )
            self.goal_index += 1
        if self.goal_index >= len(self.goals):
            self.finished = True
            self.get_logger().info('Todos os 10 Goals alcancados. Husky parado.')

    def publish_velocity(self, vx, wz):
        if self.stopping and (vx != 0 or wz != 0):
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    def front_clearance(self, heading=0.0, max_depth=3.0):
        """X minimo de obstaculo dentro da faixa varrida (metros em base_link).

        heading: orientacao hipotetica em relacao a frente atual do Husky.
        A faixa considera meia largura do robo + margem, mas NAO valida
        trajeto completo de giro/curva para manobras complexas.
        """
        xy = self.cloud_points_xy
        if xy is None or xy.shape[0] == 0:
            return float(max_depth)
        c, s = math.cos(heading), math.sin(heading)
        x_proj = c * xy[:, 0] + s * xy[:, 1]
        y_proj = -s * xy[:, 0] + c * xy[:, 1]
        corridor = ((x_proj >= 0.0) & (x_proj <= max_depth)
                    & (np.abs(y_proj) <= self.robot_half_width))
        if not np.any(corridor):
            return float(max_depth)
        return float(np.min(x_proj[corridor]))

    def rotation_area_clear(self):
        """Verifica obstaculos detectados no disco varrido pelo giro.

        Como PointCloud2 pode ter oclusoes e campo de visao limitado,
        ausencia de pontos NAO garante espaco livre absoluto.
        """
        xy = self.cloud_points_xy
        if xy is None or xy.shape[0] == 0:
            return True
        distances = np.hypot(xy[:, 0], xy[:, 1])
        return bool(np.all(distances >= self.rotation_clearance_radius))

    def choose_avoid_heading(self, gps_error):
        """Retorna angulo de giro local seguro por criterio de faixa frontal.

        Nao escolhe manobra se nenhum setor estiver livre. Os candidatos
        evitam repeticoes agressivas de giro em 90 graus ou mais.
        """
        if not self.rotation_area_clear():
            return None
        best = None
        best_score = -float('inf')
        for deg in (-60, -45, -30, 30, 45, 60):
            heading = math.radians(deg)
            clear = self.front_clearance(heading)
            if clear < self.avoid_candidate_min_clearance:
                continue
            # Prefere rota mais livre, com leve viés para o destino GPS
            # e para pequenos giros.
            heading_delta_goal = abs(self.wrap(gps_error - heading))
            score = clear - 0.60 * heading_delta_goal - 0.002 * abs(deg)
            if score > best_score:
                best_score = score
                best = heading
        return best

    def begin_avoid(self, gps_error, reset_timer=False):
        if self.sim_time_sec is None:
            self.set_blocked('Sem timestamp valido de simulacao na odometria.')
            return
        if reset_timer:
            self.avoid_started_sim = self.sim_time_sec
            self.avoid_started_wall = time.monotonic()
            self.avoid_attempts = 0
        self.avoid_attempts += 1
        self.avoid_clear_since = None
        self.avoid_traveled_odom = 0.0
        self.avoid_last_odom_xy = None
        if self.avoid_attempts > int(self.max_avoid_attempts):
            self.set_blocked('Numero maximo de tentativas de desvio atingido.')
            return
        heading = self.choose_avoid_heading(gps_error)
        if heading is None:
            self.set_blocked('Nenhum corredor validado / giro inseguro.')
            return
        self.avoid_selected_angle = heading
        self.avoid_target_yaw = self.wrap(self.yaw + heading)
        self.avoid_start_xy = (self.x, self.y)
        self.state = 'AVOID_TURN'
        self.stop()
        self.get_logger().warn(
            f'AVOID_TURN | tentativa {self.avoid_attempts} | '
            f'direcao={math.degrees(heading):+.0f}° | '
            f'Goal {self.goal_index + 1}/{len(self.goals)} preservado.'
        )

    def set_blocked(self, reason):
        self.state = 'BLOCKED'
        self.stop()
        self.get_logger().error(f'BLOCKED: {reason} Reiniciar apos inspecao.')

    def avoidance_elapsed_sim(self):
        if self.sim_time_sec is None or self.avoid_started_sim is None:
            return None
        return self.sim_time_sec - self.avoid_started_sim

    def log_state(self, distance=None, gps_error=None, front=None):
        now = time.monotonic()
        if now - self.last_log_at < self.log_period:
            return
        self.last_log_at = now
        if distance is None or gps_error is None:
            self.get_logger().info(f'Estado={self.state}')
            return
        info = (f'{self.state} | Goal {self.goal_index + 1}/{len(self.goals)} | '
                f'dist_goal={distance:.2f}m | '
                f'erro={math.degrees(gps_error):+.1f}°')
        if front is not None:
            info += f' | LiDAR frente={front:.2f}m'
        if self.state in ('AVOID_TURN', 'AVOID_FORWARD'):
            elapsed_sim = self.avoidance_elapsed_sim()
            if elapsed_sim is not None:
                info += f' | tempo_sim={elapsed_sim:.1f}/{self.max_avoid_seconds:.0f}s'
        if self.state == 'AVOID_FORWARD':
            goal_front = self.front_clearance(gps_error)
            rotation_ok = self.rotation_area_clear()
            info += (f' | percurso_odom={self.avoid_traveled_odom:.2f}/'
                     f'{self.avoid_min_travel:.2f}m'
                     f' | direcao_goal_livre={goal_front:.2f}m'
                     f' | giro_livre={rotation_ok}'
                     f' | frente_livre={front >= self.avoid_release_distance}')
        self.get_logger().info(info)

    def control_loop(self):
        if self.stopping:
            return
        self.sensor_heartbeat()
        if not self.inputs_ready():
            self.stop()
            self.warn_throttled(
                'Controle parado. ' + self.sensor_diagnostics()
            )
            return

        self.advance_goals()    # Atualiza Goals mesmo durante o desvio.
        if self.finished or self.state == 'BLOCKED':
            self.stop()
            return

        distance, gps_error = self.goal_error()
        front = self.front_clearance() if self.enable_avoidance else None
        self.log_state(distance, gps_error, front)

        # Protecao prioritaria: evita comandar avanço com obstaculo muito perto.
        if self.enable_avoidance and front < self.emergency_stop_distance:
            self.set_blocked(
                f'Obstaculo muito proximo ({front:.2f}m do base_link).'
            )
            return

        if self.state == 'GPS_NAVIGATION':
            if self.enable_avoidance and front <= self.obstacle_trigger_distance:
                self.begin_avoid(gps_error, reset_timer=True)
                return
            vx = self.clamp(self.k_linear * distance, 0.0, self.max_linear_speed)
            wz = self.clamp(
                self.k_angular * gps_error,
                -self.max_angular_speed, self.max_angular_speed,
            )
            if abs(gps_error) > math.radians(60.0):
                vx = 0.0
            elif abs(gps_error) > math.radians(30.0):
                vx *= 0.4
            self.publish_velocity(vx, wz)
            return

        if self.state == 'AVOID_TURN':
            elapsed_sim = self.avoidance_elapsed_sim()
            if elapsed_sim is None or elapsed_sim < -0.05:
                self.set_blocked('Timestamp de simulacao invalido durante giro.')
                return
            if elapsed_sim >= self.max_avoid_seconds:
                self.set_blocked(
                    f'Tempo maximo de SIMULACAO no giro: {elapsed_sim:.1f}s.'
                )
                return
            if (self.avoid_started_wall is None or
                    time.monotonic() - self.avoid_started_wall >= self.max_avoid_wall_seconds):
                self.set_blocked('Watchdog real de giro/desvio atingido.')
                return
            if not self.rotation_area_clear():
                self.set_blocked('Obstaculo dentro da area de varredura do giro.')
                return
            error = self.wrap(self.avoid_target_yaw - self.yaw)
            if abs(error) <= math.radians(self.avoid_turn_tolerance_deg):
                self.state = 'AVOID_FORWARD'
                self.avoid_start_xy = (self.x, self.y)
                self.avoid_last_odom_xy = (self.x, self.y)
                self.avoid_traveled_odom = 0.0
                self.avoid_clear_since = None
                self.get_logger().info('Giro concluido; iniciando passagem monitorada.')
                self.stop()
                return
            wz = self.avoid_turn_speed if error > 0 else -self.avoid_turn_speed
            self.publish_velocity(0.0, wz)
            return

        if self.state == 'AVOID_FORWARD':
            # Limite principal em TEMPO DE SIMULACAO, retirado do header da
            # odometria (nao do tempo real do computador).
            elapsed_sim = self.avoidance_elapsed_sim()
            if elapsed_sim is None or elapsed_sim < -0.05:
                self.set_blocked('Timestamp de simulacao invalido durante desvio.')
                return
            if elapsed_sim >= self.max_avoid_seconds:
                self.set_blocked(
                    f'Tempo maximo de SIMULACAO no desvio: {elapsed_sim:.1f}s; '
                    f'percurso_odom={self.avoid_traveled_odom:.2f}m.'
                )
                return
            # Watchdog independente do Gazebo para prevenir espera ilimitada
            # caso o relogio de simulacao pare de progredir.
            if (self.avoid_started_wall is None or
                    time.monotonic() - self.avoid_started_wall >= self.max_avoid_wall_seconds):
                self.set_blocked('Watchdog de tempo real do desvio atingido.')
                return
            if front <= self.avoid_forward_stop_distance:
                self.stop()
                self.begin_avoid(gps_error)
                return

            traveled = self.avoid_traveled_odom
            # O Goal pode estar atras ou ao lado da arvore. Checar tambem
            # a direcao para a qual o GPS vai tentar reorientar o Husky.
            goal_front = self.front_clearance(gps_error)
            rotation_ok = self.rotation_area_clear()
            can_release = (
                traveled >= self.avoid_min_travel
                and front >= self.avoid_release_distance
                and goal_front >= self.avoid_release_distance
                and rotation_ok
            )
            if can_release:
                if self.avoid_clear_since is None:
                    self.avoid_clear_since = self.sim_time_sec
                if self.sim_time_sec - self.avoid_clear_since >= self.avoid_hold_seconds:
                    self.state = 'GPS_NAVIGATION'
                    self.avoid_attempts = 0
                    self.get_logger().info(
                        f'Passagem monitorada concluida: percurso_odom={traveled:.2f}m, '
                        f'frente={front:.2f}m, direcao_goal={goal_front:.2f}m, '
                        f'tempo_sim={elapsed_sim:.1f}s. Retomando orientacao ao Goal.'
                    )
                    self.stop()
                    return
            else:
                self.avoid_clear_since = None

            # Avanca lentamente apenas enquanto o corredor ATUAL estiver livre.
            # Se o retorno ao Goal nao for validado, prossegue na direcao de
            # desvio ate liberar o setor ou esgotar os limites de seguranca.
            heading_err = self.wrap(self.avoid_target_yaw - self.yaw)
            wz = self.clamp(
                1.0 * heading_err,
                -self.avoid_turn_speed, self.avoid_turn_speed,
            )
            self.publish_velocity(self.avoid_speed, wz)
            return

        self.set_blocked(f'Estado desconhecido: {self.state}')

    def shutdown_stop(self):
        if self.stopping:
            return
        self.stopping = True
        self.timer.cancel()
        self.get_logger().warn('Ctrl+C: publicando 30 comandos ZERO em /platform/cmd_vel.')
        sent = 0
        for _ in range(30):
            if not rclpy.ok():
                break
            try:
                self.stop()
                sent += 1
                time.sleep(0.05)
            except Exception as exc:
                self.get_logger().error(f'Falha durante parada: {exc}')
                break
        self.get_logger().info(f'{sent} comandos ZERO publicados (confirmar parada no Gazebo).')

    def handle_sigint(self, _signum, _frame):
        if self.stopping:
            return
        self.shutdown_stop()
        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AutonomousNavigationNode()
    previous_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, node.handle_sigint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_stop()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
