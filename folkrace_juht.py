#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


SOIDUKIIRUS = 0.30
AEGLANE_KIIRUS = 0.16
VAGA_AEGLANE_KIIRUS = 0.08

POORDEKIIRUS = 0.9

OHUPIIR_ETTE = 0.55
OHUPIIR_KULG = 0.28


class FolkraceJuht(Node):
    def __init__(self):
        super().__init__('folkrace_juht')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_cb,
            10
        )

        self.viimane_olek = "ootan lidar andmeid"

    def _olek(self, tekst):
        self.viimane_olek = tekst
        self.get_logger().info(tekst)

    def _sektori_min(self, ranges, nurk_min, nurk_max, default=8.0):
        """
        Leiab sektori kauguse lidarist.
        0 kraadi = ette, +90 = vasak, -90 = parem.
        """
        n = len(ranges)
        if n == 0:
            return default

        values = []

        for kraad in range(nurk_min, nurk_max + 1):
            # LaserScan: -180 kraadi on index 0, 0 kraadi on index 360
            index = int((kraad + 180) / 360 * n) % n
            r = ranges[index]

            if math.isfinite(r) and 0.12 < r < 8.0:
                values.append(r)

        if len(values) == 0:
            return default

        # Mediaan teeb lugemise stabiilsemaks kui yksik min vaartus
        return statistics.median(values)

    def _scan_cb(self, msg):
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

        self.pub.publish(cmd)

    def _arvuta_kiirus(self, ette, ette_vasak, ette_parem, vasak, parem):
        """
        Lidaripohine reaktiivne juhtimine.

        Tagastab:
        kiirus - linear.x
        poore  - angular.z
        """

        # Arvesta ka diagonaalseid sektoreid, et robot aeglustaks enne kurvi.
        ohu_kaugus = min(ette, ette_vasak * 0.85, ette_parem * 0.85)

        # Kiiruse reguleerimine:
        # kaugel = kiire, lahedal = aeglane
        if ohu_kaugus > 1.2:
            kiirus = SOIDUKIIRUS
        elif ohu_kaugus > OHUPIIR_ETTE:
            kiirus = AEGLANE_KIIRUS
        else:
            kiirus = VAGA_AEGLANE_KIIRUS

        # Raja keskel hoidmine.
        # Kui vasakul on rohkem ruumi kui paremal, poora vasakule.
        # Kui paremal on rohkem ruumi kui vasakul, poora paremale.
        kylje_viga = vasak - parem
        poore = 0.45 * kylje_viga

        # Kui ees voi diagonaalis on sein lahedal, poora tugevamalt sinna,
        # kus ruumi rohkem on.
        if ette < OHUPIIR_ETTE or ette_vasak < 0.45 or ette_parem < 0.45:
            if ette_vasak > ette_parem:
                poore = POORDEKIIRUS
                self._olek("sein ees, pooran vasakule")
            else:
                poore = -POORDEKIIRUS
                self._olek("sein ees, pooran paremale")
        else:
            self._olek("soidan ja hoian rada keskel")

        # Kylgseina kaitse.
        if vasak < OHUPIIR_KULG:
            poore = -POORDEKIIRUS
            kiirus = min(kiirus, AEGLANE_KIIRUS)

        if parem < OHUPIIR_KULG:
            poore = POORDEKIIRUS
            kiirus = min(kiirus, AEGLANE_KIIRUS)

        # Piira poore, et robot ei pendeldaks liiga palju.
        poore = max(-POORDEKIIRUS, min(POORDEKIIRUS, poore))

        return kiirus, poore

    def peata(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FolkraceJuht()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.peata()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
