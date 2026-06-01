"""
EKF tabanlı sensör füzyonu: IMU + GNSS + opsiyonel VSLAM
Durum vektörü (9): [x, y, z, vx, vy, vz, roll, pitch, yaw]  (metre, m/s, radyan)
Konum GPS referansına göre metre cinsinden tutulur, çıkışta GPS'e çevrilir.

Gerçek donanım gelince:
  - IMU: Pixhawk IMU verisi MAVLink SCALED_IMU mesajından
  - GNSS: MAVLink GPS_RAW_INT mesajından
  - VSLAM: ORB-SLAM3 ROS2 topic'inden (opsiyonel)
"""

import math
import time
import random
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np


# ── Veri yapıları ──────────────────────────────────────────────

@dataclass
class IMUVeri:
    ax: float   # m/s²  (ileri)
    ay: float   # m/s²  (sağ)
    az: float   # m/s²  (aşağı — yerçekimi dahil)
    gx: float   # rad/s (roll hızı)
    gy: float   # rad/s (pitch hızı)
    gz: float   # rad/s (yaw hızı)
    zaman: float = field(default_factory=time.time)


@dataclass
class GNSSVeri:
    lat: float        # derece
    lon: float        # derece
    alt: float        # metre (MSL)
    hdop: float = 1.0 # yatay hassasiyet faktörü (1=iyi, 5=kötü)
    zaman: float = field(default_factory=time.time)


@dataclass
class VSLAMVeri:
    x: float    # metre (referansa göre)
    y: float
    z: float
    roll: float   # radyan
    pitch: float
    yaw: float
    guven: float = 1.0  # 0-1 arası güven skoru
    zaman: float = field(default_factory=time.time)


@dataclass
class FuzyonCikti:
    lat: float
    lon: float
    alt: float
    vx: float   # m/s
    vy: float
    vz: float
    roll: float   # radyan
    pitch: float
    yaw: float
    konum_belirsizlik: float  # metre (1-sigma)
    kaynak: str   # "imu+gnss" / "imu+gnss+vslam" / "imu_only"


# ── Koordinat yardımcıları ──────────────────────────────────────

def gps_to_metre(lat, lon, ref_lat, ref_lon):
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * 6371000
    y = math.radians(lat - ref_lat) * 6371000
    return x, y


def metre_to_gps(x, y, ref_lat, ref_lon):
    lat = ref_lat + math.degrees(y / 6371000)
    lon = ref_lon + math.degrees(x / (6371000 * math.cos(math.radians(ref_lat))))
    return lat, lon


# ── EKF Sensör Füzyonu ─────────────────────────────────────────

class SensorFuzyon:
    """
    9-durumlu EKF: konum (x,y,z), hız (vx,vy,vz), açı (roll,pitch,yaw)
    IMU → tahmin adımı (yüksek frekans)
    GNSS → ölçüm güncellemesi (düşük frekans)
    VSLAM → ek ölçüm güncellemesi (opsiyonel)
    """

    YERCEKIMI = 9.81  # m/s²

    # Süreç gürültüsü (Q) — sensör ne kadar güvenilir
    Q_KONUM  = 0.01   # m²
    Q_HIZ    = 0.1    # (m/s)²
    Q_ACI    = 0.001  # rad²

    # GNSS ölçüm gürültüsü (R)
    R_GNSS_XY  = 4.0   # m² (tipik GPS ~2m hata)
    R_GNSS_Z   = 9.0   # m²

    # VSLAM ölçüm gürültüsü
    R_VSLAM_KONUM = 0.04  # m²
    R_VSLAM_ACI   = 0.01  # rad²

    def __init__(self, baslangic_lat: float, baslangic_lon: float, baslangic_alt: float = 0.0):
        self.ref_lat = baslangic_lat
        self.ref_lon = baslangic_lon

        # Durum vektörü x = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.x = np.zeros(9)
        self.x[2] = baslangic_alt

        # Kovaryans matrisi P
        self.P = np.diag([1.0, 1.0, 2.0,    # konum belirsizliği
                          0.5, 0.5, 0.5,    # hız belirsizliği
                          0.1, 0.1, 0.5])   # açı belirsizliği

        # Süreç gürültüsü Q
        self.Q = np.diag([
            self.Q_KONUM, self.Q_KONUM, self.Q_KONUM,
            self.Q_HIZ,   self.Q_HIZ,   self.Q_HIZ,
            self.Q_ACI,   self.Q_ACI,   self.Q_ACI,
        ])

        self._son_imu_zaman: Optional[float] = None
        self._gnss_alinan = False
        self._vslam_alinan = False

    # ── Tahmin adımı (IMU) ─────────────────────────────────────

    def imu_guncelle(self, veri: IMUVeri):
        if self._son_imu_zaman is None:
            self._son_imu_zaman = veri.zaman
            return

        dt = veri.zaman - self._son_imu_zaman
        if dt <= 0 or dt > 0.5:  # aşırı büyük dt'yi atla
            self._son_imu_zaman = veri.zaman
            return
        self._son_imu_zaman = veri.zaman

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]

        # Dünya çerçevesinde ivme (yerçekimi çıkar, basit rotasyon)
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        # Rotasyon matrisi (gövde → dünya) — ZYX Euler
        ax_d = (cy*cp)*veri.ax + (cy*sp*sr - sy*cr)*veri.ay + (cy*sp*cr + sy*sr)*veri.az
        ay_d = (sy*cp)*veri.ax + (sy*sp*sr + cy*cr)*veri.ay + (sy*sp*cr - cy*sr)*veri.az
        az_d =   (-sp)*veri.ax +            (cp*sr)*veri.ay +            (cp*cr)*veri.az - self.YERCEKIMI

        # Durum geçiş — sabit ivme modeli
        F = np.eye(9)
        F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt  # konum += hız*dt

        self.x[0] += self.x[3] * dt + 0.5 * ax_d * dt**2
        self.x[1] += self.x[4] * dt + 0.5 * ay_d * dt**2
        self.x[2] += self.x[5] * dt + 0.5 * az_d * dt**2
        self.x[3] += ax_d * dt
        self.x[4] += ay_d * dt
        self.x[5] += az_d * dt

        # Gyro ile açı entegrasyonu
        self.x[6] += veri.gx * dt
        self.x[7] += veri.gy * dt
        self.x[8] += veri.gz * dt
        self.x[8] = (self.x[8] + math.pi) % (2 * math.pi) - math.pi  # yaw'ı [-π, π]'a sar

        # Kovaryans tahmin
        self.P = F @ self.P @ F.T + self.Q * dt

    # ── GNSS ölçüm güncellemesi ────────────────────────────────

    def gnss_guncelle(self, veri: GNSSVeri):
        mx, my = gps_to_metre(veri.lat, veri.lon, self.ref_lat, self.ref_lon)

        # Ölçüm matrisi H: konum x,y,z'yi gözlemliyoruz
        H = np.zeros((3, 9))
        H[0, 0] = 1.0; H[1, 1] = 1.0; H[2, 2] = 1.0

        hdop2 = max(1.0, veri.hdop) ** 2
        R = np.diag([self.R_GNSS_XY * hdop2,
                     self.R_GNSS_XY * hdop2,
                     self.R_GNSS_Z])

        z = np.array([mx, my, veri.alt])
        self._kalman_guncelle(H, R, z)
        self._gnss_alinan = True

    # ── VSLAM ölçüm güncellemesi ───────────────────────────────

    def vslam_guncelle(self, veri: VSLAMVeri):
        if veri.guven < 0.3:  # düşük güven → atla
            return

        # VSLAM: konum (x,y,z) + yaw ölçümü (roll/pitch IMU'dan yeterli)
        H = np.zeros((4, 9))
        H[0, 0] = 1.0; H[1, 1] = 1.0; H[2, 2] = 1.0; H[3, 8] = 1.0

        guven_carpan = 1.0 / max(0.1, veri.guven)
        R = np.diag([
            self.R_VSLAM_KONUM * guven_carpan,
            self.R_VSLAM_KONUM * guven_carpan,
            self.R_VSLAM_KONUM * guven_carpan,
            self.R_VSLAM_ACI   * guven_carpan,
        ])

        z = np.array([veri.x, veri.y, veri.z, veri.yaw])
        self._kalman_guncelle(H, R, z)
        self._vslam_alinan = True

    # ── Kalman güncelleme çekirdeği ────────────────────────────

    def _kalman_guncelle(self, H, R, z):
        y  = z - H @ self.x                           # inovasyon
        S  = H @ self.P @ H.T + R                     # inovasyon kovaryansı
        K  = self.P @ H.T @ np.linalg.inv(S)          # Kalman kazancı
        self.x = self.x + K @ y
        self.P = (np.eye(9) - K @ H) @ self.P

    # ── Mevcut durumu al ───────────────────────────────────────

    def durum_al(self) -> FuzyonCikti:
        lat, lon = metre_to_gps(self.x[0], self.x[1], self.ref_lat, self.ref_lon)
        belirsizlik = math.sqrt(self.P[0, 0] + self.P[1, 1])

        if self._vslam_alinan:
            kaynak = "imu+gnss+vslam"
        elif self._gnss_alinan:
            kaynak = "imu+gnss"
        else:
            kaynak = "imu_only"

        return FuzyonCikti(
            lat=round(lat, 8),
            lon=round(lon, 8),
            alt=round(self.x[2], 2),
            vx=round(self.x[3], 3),
            vy=round(self.x[4], 3),
            vz=round(self.x[5], 3),
            roll=round(self.x[6], 4),
            pitch=round(self.x[7], 4),
            yaw=round(self.x[8], 4),
            konum_belirsizlik=round(belirsizlik, 3),
            kaynak=kaynak,
        )


# ── Mock sensör üreticisi ──────────────────────────────────────

class MockSensorler:
    """Donanım olmadan gerçekçi sensör verisi üretir."""

    def __init__(self, baslangic_lat=37.5730, baslangic_lon=36.9180, baslangic_alt=100.0):
        self.lat = baslangic_lat
        self.lon = baslangic_lon
        self.alt = baslangic_alt
        self.yaw = 0.45   # ~25 derece
        self._t = 0.0

    def imu_oku(self, dt=0.01) -> IMUVeri:
        self._t += dt
        # Hafif yatay uçuş + gürültü
        return IMUVeri(
            ax=0.1  + random.gauss(0, 0.05),
            ay=0.02 + random.gauss(0, 0.03),
            az=9.81 + random.gauss(0, 0.1),   # yerçekimi + gürültü
            gx=random.gauss(0, 0.002),
            gy=random.gauss(0, 0.002),
            gz=random.gauss(0, 0.001),
            zaman=time.time(),
        )

    def gnss_oku(self) -> GNSSVeri:
        # GPS ~2m hata simülasyonu
        return GNSSVeri(
            lat=self.lat  + random.gauss(0, 0.000018),
            lon=self.lon  + random.gauss(0, 0.000018),
            alt=self.alt  + random.gauss(0, 3.0),
            hdop=random.uniform(1.0, 1.8),
            zaman=time.time(),
        )

    def vslam_oku(self) -> VSLAMVeri:
        # VSLAM ~5cm hata — GPS'ten çok daha hassas
        return VSLAMVeri(
            x=0.0 + random.gauss(0, 0.05),
            y=0.0 + random.gauss(0, 0.05),
            z=self.alt + random.gauss(0, 0.05),
            roll=random.gauss(0, 0.002),
            pitch=random.gauss(0, 0.002),
            yaw=self.yaw + random.gauss(0, 0.005),
            guven=random.uniform(0.85, 1.0),
            zaman=time.time(),
        )


# ── Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    print("=" * 55)
    print("  KIZIL-2 Sensör Füzyonu (EKF) — Mock Test")
    print("=" * 55)

    mock    = MockSensorler()
    fuzyon  = SensorFuzyon(mock.lat, mock.lon, mock.alt)

    ADIM_SAYISI   = 200   # 200 IMU adımı
    GNSS_ARALIK   = 10    # her 10 IMU adımında bir GPS güncelle (~10Hz IMU, ~1Hz GPS)
    VSLAM_ARALIK  = 5     # her 5 adımda VSLAM

    print(f"\nBaşlangıç: {mock.lat:.6f}, {mock.lon:.6f}, {mock.alt:.1f}m\n")
    print(f"{'Adım':>5} | {'Lat':>12} | {'Lon':>12} | {'Alt':>7} | {'Belirsiz':>9} | {'Kaynak'}")
    print("-" * 70)

    for i in range(ADIM_SAYISI):
        # IMU her adımda
        fuzyon.imu_guncelle(mock.imu_oku(dt=0.01))

        # GPS seyrek
        if i % GNSS_ARALIK == 0:
            fuzyon.gnss_guncelle(mock.gnss_oku())

        # VSLAM opsiyonel
        if i % VSLAM_ARALIK == 0:
            fuzyon.vslam_guncelle(mock.vslam_oku())

        # Her 50 adımda rapor
        if i % 50 == 0:
            d = fuzyon.durum_al()
            print(f"{i:>5} | {d.lat:>12.7f} | {d.lon:>12.7f} | {d.alt:>6.1f}m | "
                  f"±{d.konum_belirsizlik:>6.3f}m | {d.kaynak}")

    print("\nSon durum:")
    son = fuzyon.durum_al()
    print(f"  Konum     : {son.lat:.7f}, {son.lon:.7f}, {son.alt:.1f}m")
    print(f"  Hız       : vx={son.vx:.3f} vy={son.vy:.3f} vz={son.vz:.3f} m/s")
    print(f"  Açı       : roll={math.degrees(son.roll):.1f}° pitch={math.degrees(son.pitch):.1f}° yaw={math.degrees(son.yaw):.1f}°")
    print(f"  Belirsizlik: ±{son.konum_belirsizlik:.3f}m")
    print(f"  Kaynak    : {son.kaynak}")
    print("\nEKF füzyonu başarılı.")
