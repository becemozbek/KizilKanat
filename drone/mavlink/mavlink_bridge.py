"""
Jetson <-> Pixhawk MAVLink köprüsü
Gerçek donanım olmadan SITL ile test edilebilir.
"""
from pymavlink import mavutil
import time

SITL_BAĞLANTI   = "udp:127.0.0.1:14550"   # SITL için
PIXHAWK_BAĞLANTI = "/dev/ttyUSB0"          # Gerçek Pixhawk için
PIXHAWK_BAUD     = 57600


class MAVLinkKöprü:
    def __init__(self, sitl_modu=True):
        bağlantı = SITL_BAĞLANTI if sitl_modu else PIXHAWK_BAĞLANTI
        baud     = None           if sitl_modu else PIXHAWK_BAUD
        print(f"[MAVLink] Bağlanıyor: {bağlantı}")
        self.master = mavutil.mavlink_connection(bağlantı, baud=baud)
        self.master.wait_heartbeat()
        print("[MAVLink] Heartbeat alındı — bağlantı kuruldu.")

    def mod_değiştir(self, mod: str):
        """GUIDED, LOITER, RTL vb."""
        self.master.set_mode(mod)
        print(f"[MAVLink] Mod: {mod}")

    def waypoint_gönder(self, lat: float, lon: float, alt: float):
        self.master.mav.mission_item_int_send(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 1, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), alt
        )
        print(f"[MAVLink] Waypoint gönderildi: {lat}, {lon}, {alt}m")

    def telemetri_oku(self):
        msg = self.master.recv_match(type="VFR_HUD", blocking=True, timeout=2)
        if msg:
            return {
                "irtifa": msg.alt,
                "hiz":    msg.groundspeed,
                "bas":    msg.heading,
            }
        return None

    def arm(self):
        self.master.arducopter_arm()
        print("[MAVLink] ARM komutu gönderildi.")

    def disarm(self):
        self.master.arducopter_disarm()
        print("[MAVLink] DISARM komutu gönderildi.")


if __name__ == "__main__":
    köprü = MAVLinkKöprü(sitl_modu=True)
    while True:
        veri = köprü.telemetri_oku()
        if veri:
            print(f"İrtifa: {veri['irtifa']:.1f}m | Hız: {veri['hiz']:.1f} m/s")
        time.sleep(1)
