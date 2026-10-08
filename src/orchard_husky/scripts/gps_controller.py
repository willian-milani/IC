#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class GPSController(Node):

    def __init__(self):
        super().__init__('gps_controller')

        # ==========================================================
        # TÓPICOS
        # ==========================================================

        self.gps_topic = '/sensors/gps/navsat'
        self.odom_topic = '/platform/odom'
        self.cmd_vel_topic = '/platform/cmd_vel'

        # ==========================================================
        # CONFIGURAÇÃO DO CONTROLADOR
        # ==========================================================

        self.k_linear = 0.30
        self.k_angular = 1.50

        self.max_linear_speed = 0.13
        self.max_angular_speed = 0.45

        # Distância para considerar o waypoint alcançado
        self.goal_tolerance = 0.50

        # ==========================================================
        # WAYPOINTS GPS
        # ==========================================================
        #
        # (latitude, longitude)
        #
        # ATENÇÃO:
        # coloque aqui as coordenadas dos pontos desejados.
        #

        self.goals = [

            # ======================================================
            # CURVA 1 - DIREITA
            # ======================================================

            (-15.00010790337, -46.99980753637),
            (-15.00010363639, -46.99979539986),
            (-15.00009561447, -46.99978550465),
            (-15.00008480779, -46.99977904115),
            (-15.00007251889, -46.99977679985),  # ápice
            (-15.00006022999, -46.99977904115),
            (-15.00004942331, -46.99978550465),
            (-15.00004140139, -46.99979539986),
            (-15.00003713441, -46.99980753637),

            # ======================================================
            # RETA 1 - REFERÊNCIAS INTERMEDIÁRIAS
            # ======================================================

            (-15.00003700202, -46.99985403640),  # ~5 m
            (-15.00003660484, -46.99999353648),  # ~20 m
            (-15.00003620767, -47.00013303656),  # ~35 m

            # ======================================================
            # CURVA 2 - ESQUERDA
            # ======================================================

            (-15.00003603847, -47.00019246363),
            (-15.00003177149, -47.00020460014),
            (-15.00002374958, -47.00021449535),
            (-15.00001294289, -47.00022095885),
            (-15.00000065400, -47.00022320015),  # ápice
            (-14.99998836510, -47.00022095885),
            (-14.99997755842, -47.00021449535),
            (-14.99996953650, -47.00020460014),
            (-14.99996526952, -47.00019246363),

            # ======================================================
            # RETA 2 - REFERÊNCIAS INTERMEDIÁRIAS
            # ======================================================

            (-14.99996526952, -47.00014596342),  # ~5 m
            (-14.99996526952, -47.00000646278),  # ~20 m
            (-14.99996526952, -46.99986696214),  # ~35 m

            # ======================================================
            # CURVA 3 - DIREITA
            # ======================================================

            (-14.99996526952, -46.99980753637),
            (-14.99996055338, -46.99979539986),
            (-14.99995188468, -46.99978550465),
            (-14.99994107800, -46.99977904115),
            (-14.99992878910, -46.99977679985),  # ápice
            (-14.99991650020, -46.99977904115),
            (-14.99990569352, -46.99978550465),
            (-14.99989767160, -46.99979539986),
            (-14.99989340462, -46.99980753637),

            # ======================================================
            # RETA 3 - REFERÊNCIAS INTERMEDIÁRIAS
            # ======================================================

            (-14.99989333729, -46.99985403652),  # ~5 m
            (-14.99989313531, -46.99999353695),  # ~20 m
            (-14.99989293333, -47.00013303739),  # ~35 m

            # ======================================================
            # FINAL - CAMINHO 4
            # ======================================================

            (-14.99989285665, -47.00018600013),
        ]
        self.current_goal = 0
        self.finished = False

        # ==========================================================
        # ESTADO DO ROBÔ
        # ==========================================================

        self.latitude = None
        self.longitude = None
        self.yaw = None

        # Origem do sistema local
        self.origin_latitude = None
        self.origin_longitude = None

        # ==========================================================
        # SUBSCRIBERS
        # ==========================================================

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # ==========================================================
        # PUBLISHER
        # ==========================================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )

        # ==========================================================
        # LOOP DE CONTROLE
        # ==========================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Controlador GPS iniciado.'
        )

    # ==============================================================
    # GPS
    # ==============================================================

    def gps_callback(self, msg):

        # Verifica se existe uma posição válida
        if msg.status.status < 0:
            self.get_logger().warn(
                'GPS sem fix válido.'
            )
            return

        self.latitude = msg.latitude
        self.longitude = msg.longitude

        # ----------------------------------------------------------
        # Primeira posição recebida vira a origem local
        # ----------------------------------------------------------

        if self.origin_latitude is None:

            self.origin_latitude = self.latitude
            self.origin_longitude = self.longitude

            self.get_logger().info(
                f'Origem GPS definida: '
                f'lat={self.origin_latitude:.10f}, '
                f'lon={self.origin_longitude:.10f}'
            )

    # ==============================================================
    # ODOMETRIA
    # ==============================================================

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        # Quaternion -> yaw

        siny = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny,
            cosy
        )

    # ==============================================================
    # CONVERSÃO GPS -> METROS
    # ==============================================================

    def gps_to_local(self, latitude, longitude):

        # Raio aproximado da Terra
        earth_radius = 6378137.0

        lat0 = math.radians(
            self.origin_latitude
        )

        # Diferenças angulares
        d_lat = math.radians(
            latitude - self.origin_latitude
        )

        d_lon = math.radians(
            longitude - self.origin_longitude
        )

        # ----------------------------------------------------------
        # x = leste
        # y = norte
        # ----------------------------------------------------------

        x = (
            d_lon *
            earth_radius *
            math.cos(lat0)
        )

        y = (
            d_lat *
            earth_radius
        )

        return x, y

    # ==============================================================
    # CONTROLE
    # ==============================================================

    def control_loop(self):

        # Ainda não temos GPS
        if self.latitude is None:
            return

        # Ainda não temos orientação
        if self.yaw is None:
            return

        # Ainda não temos origem
        if self.origin_latitude is None:
            return

        # ----------------------------------------------------------
        # TODOS OS GOALS ALCANÇADOS
        # ----------------------------------------------------------

        if self.finished:

            self.publish_velocity(
                0.0,
                0.0
            )

            return

        # ----------------------------------------------------------
        # VERIFICAÇÃO DO ÍNDICE
        # ----------------------------------------------------------

        if self.current_goal >= len(self.goals):

            self.finished = True

            self.publish_velocity(
                0.0,
                0.0
            )

            self.get_logger().info(
                'Todos os goals foram alcançados.'
            )

            return

        # ----------------------------------------------------------
        # POSIÇÃO ATUAL
        # ----------------------------------------------------------

        current_x, current_y = self.gps_to_local(
            self.latitude,
            self.longitude
        )

        # ----------------------------------------------------------
        # GOAL ATUAL
        # ----------------------------------------------------------

        goal_lat, goal_lon = self.goals[
            self.current_goal
        ]

        goal_x, goal_y = self.gps_to_local(
            goal_lat,
            goal_lon
        )

        # ----------------------------------------------------------
        # ERRO DE POSIÇÃO
        # ----------------------------------------------------------

        dx = goal_x - current_x
        dy = goal_y - current_y

        distance = math.hypot(
            dx,
            dy
        )

        # ----------------------------------------------------------
        # DIREÇÃO DO GOAL
        # ----------------------------------------------------------

        angle_to_goal = math.atan2(
            dy,
            dx
        )

        # ----------------------------------------------------------
        # ERRO ANGULAR
        # ----------------------------------------------------------

        angle_error = self.normalize_angle(
            angle_to_goal - self.yaw
        )

        # ----------------------------------------------------------
        # MOSTRA INFORMAÇÕES
        # ----------------------------------------------------------

        self.get_logger().info(
            f'Goal {self.current_goal} | '
            f'dist={distance:.2f} m | '
            f'erro_angular={math.degrees(angle_error):.1f}°'
        )

        # ----------------------------------------------------------
        # GOAL ALCANÇADO
        # ----------------------------------------------------------

        if distance < self.goal_tolerance:

            self.get_logger().info(
                f'GOAL {self.current_goal} ALCANÇADO!'
            )

            self.current_goal += 1

            self.publish_velocity(
                0.0,
                0.0
            )

            return

        # ----------------------------------------------------------
        # CONTROLE PROPORCIONAL
        # ----------------------------------------------------------

        linear_velocity = (
            self.k_linear *
            distance
        )

        angular_velocity = (
            self.k_angular *
            angle_error
        )

        # ----------------------------------------------------------
        # LIMITES
        # ----------------------------------------------------------

        linear_velocity = max(
            0.0,
            min(
                linear_velocity,
                self.max_linear_speed
            )
        )

        angular_velocity = max(
            -self.max_angular_speed,
            min(
                angular_velocity,
                self.max_angular_speed
            )
        )

        # ----------------------------------------------------------
        # REDUZ VELOCIDADE QUANDO ESTÁ MUITO FORA DA DIREÇÃO
        # ----------------------------------------------------------

        if abs(angle_error) > math.radians(60.0):

            linear_velocity = 0.0

        elif abs(angle_error) > math.radians(30.0):

            linear_velocity *= 0.4

        # ----------------------------------------------------------
        # ENVIA COMANDO
        # ----------------------------------------------------------

        self.publish_velocity(
            linear_velocity,
            angular_velocity
        )

    # ==============================================================
    # PUBLICA VELOCIDADE
    # ==============================================================

    def publish_velocity(
        self,
        linear,
        angular
    ):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = linear
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular

        self.cmd_pub.publish(msg)

    # ==============================================================
    # NORMALIZA ÂNGULO
    # ==============================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


def main(args=None):

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = GPSController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print('\nCtrl+C recebido.')
        print('Parando o Husky...')

    finally:

        # Envia comando de parada enquanto o ROS 2
        # ainda está ativo
        if rclpy.ok():

            for _ in range(5):

                node.publish_velocity(
                    0.0,
                    0.0
                )

                time.sleep(0.02)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()