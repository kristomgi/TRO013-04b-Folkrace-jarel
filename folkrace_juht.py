#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FolkraceJuht(Node):
    MAX_KIIRUS = 0.35
    MIN_KIIRUS = 0.10
    MAX_POORDEKIIRUS = 1.0

    OHUTU_KAUGUS = 1.0
    KRIITILINE_KAUGUS = 0.35
    KULG_OHUPIIR = 0.25

    RINGIDE_ARV = 3
    RINGI_PIKKUS = 5.0

    def __init__(self):
        super().__init__('folkrace_juht')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.praegune_x = 0.0
        self.praegune_y = 0.0
        self.prev_x = None
        self.prev_y = None

        self.kogu_labitud = 0.0
        self.labitud_ringid = 0

        self.get_logger().info('Folkrace juht kaivitatud')

    def odom_callback(self, msg):
        self.praegune_x = msg.pose.pose.position.x
        self.praegune_y = msg.pose.pose.position.y

        if self.prev_x is not None:
            dx = self.praegune_x - self.prev_x
            dy = self.praegune_y - self.prev_y
            self.kogu_labitud += math.sqrt(dx * dx + dy * dy)

            if self.kogu_labitud >= self.RINGI_PIKKUS * (self.labitud_ringid + 1):
                self.labitud_ringid += 1
                self.get_logger().info(
                    f'Ring {self.labitud_ringid}/{self.RINGIDE_ARV} labitud'
                )

        self.prev_x = self.praegune_x
        self.prev_y = self.praegune_y

    def _piira(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def _sektori_min(self, ranges, nurk_min, nurk_max, default=8.0):
        n = len(ranges)
        if n == 0:
            return default

        values = []

        for kraad in range(nurk_min, nurk_max + 1):
            index = int((kraad + 180) / 360 * n) % n
            r = ranges[index]

            if math.isfinite(r) and 0.12 < r < 8.0:
                values.append(r)

        if len(values) == 0:
            return default

        return statistics.median(values)

    def scan_callback(self, msg):
        if self.labitud_ringid >= self.RINGIDE_ARV:
            self._peata()
            return

        ranges = msg.ranges

        ette = self._sektori_min(ranges, -20, 20)
        ette_vasak = self._sektori_min(ranges, 20, 60)
        ette_parem = self._sektori_min(ranges, -60, -20)
        vasak = self._sektori_min(ranges, 60, 120)
        parem = self._sektori_min(ranges, -120, -60)

        kiirus, poore = self._arvuta_kiirus(
            ette,
            ette_vasak,
            ette_parem,
            vasak,
            parem
        )

        cmd = Twist()
        cmd.linear.x = kiirus
        cmd.angular.z = poore
        self.cmd_pub.publish(cmd)

    def _arvuta_kiirus(self, ette, ette_vasak, ette_parem, vasak, parem):
        ohu_kaugus = min(ette, ette_vasak * 0.85, ette_parem * 0.85)

        if ohu_kaugus > self.OHUTU_KAUGUS:
            kiirus = self.MAX_KIIRUS
        elif ohu_kaugus > self.KRIITILINE_KAUGUS:
            ratio = (
                (ohu_kaugus - self.KRIITILINE_KAUGUS)
                / (self.OHUTU_KAUGUS - self.KRIITILINE_KAUGUS)
            )
            kiirus = self.MIN_KIIRUS + ratio * (self.MAX_KIIRUS - self.MIN_KIIRUS)
        else:
            kiirus = 0.0

        kylje_viga = vasak - parem
        poore = kylje_viga * 0.45

        if ette < self.KRIITILINE_KAUGUS:
            if vasak > parem:
                poore = self.MAX_POORDEKIIRUS
            else:
                poore = -self.MAX_POORDEKIIRUS
            kiirus = 0.0

        elif ette_vasak < 0.45 or ette_parem < 0.45:
            if ette_vasak > ette_parem:
                poore += 0.45
            else:
                poore -= 0.45
            kiirus = min(kiirus, 0.18)

        if vasak < self.KULG_OHUPIIR:
            poore = -self.MAX_POORDEKIIRUS
            kiirus = min(kiirus, 0.15)

        if parem < self.KULG_OHUPIIR:
            poore = self.MAX_POORDEKIIRUS
            kiirus = min(kiirus, 0.15)

        # Silla/kitsa koha korral hoia kiirus madalam ja poore sujuvam.
        if vasak < 0.55 and parem < 0.55 and ette > 0.6:
            kiirus = min(kiirus, 0.16)
            poore = self._piira(poore, -0.45, 0.45)

        poore = self._piira(poore, -self.MAX_POORDEKIIRUS, self.MAX_POORDEKIIRUS)

        return kiirus, poore

    def _peata(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FolkraceJuht()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node._peata()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
