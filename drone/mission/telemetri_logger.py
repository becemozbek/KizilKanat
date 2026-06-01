"""
Uçuş telemetri kaydedici.
MAVLink'ten gelen verileri CSV + JSON'a yazar, oturum bazlı saklar.
Pixhawk gelince mavlink_bridge.py ile bağlanır, şimdilik mock çalışır.
"""

import csv
import json
import os
import time
import threading
from dataclasses import dataclass, asdict, field
from datetime import datetime
from typing import List, Optional


# ── Tek bir telemetri kaydı ────────────────────────────────────

@dataclass
class TelemetriKayit:
    zaman_damgasi: float          # unix time
    zaman_str: str                # "2025-06-01 19:32:00.123"
    lat: float
    lon: float
    alt_m: float                  # irtifa (metre, MSL)
    hiz_ms: float                 # yatay hız m/s
    dikey_hiz_ms: float
    yaw_derece: float
    pitch_derece: float
    roll_derece: float
    batarya_v: float              # Volt
    batarya_yuzde: int            # %
    mod: str                      # GUIDED / AUTO / LOITER vb.
    arm: bool
    uydu_sayisi: int
    hdop: float
    gorev_notu: str = ""          # örn: "engel tespit", "rota hesaplandı"


# ── Logger ────────────────────────────────────────────────────

class TelemetriLogger:
    """
    Thread-safe telemetri kaydedici.
    - Her oturum için ayrı klasör: logs/YYYY-MM-DD_HH-MM-SS/
    - telemetri.csv  → sürekli eklenen satırlar
    - oturum.json    → oturum özeti (kapanışta yazılır)
    - olaylar.log    → önemli olaylar (arm, mod değişimi, engel)
    """

    def __init__(self, log_koku: str = None):
        if log_koku is None:
            base = os.path.join(
                os.path.dirname(__file__), '../../logs'
            )
        else:
            base = log_koku

        oturum_ad = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.klasor = os.path.abspath(os.path.join(base, oturum_ad))
        os.makedirs(self.klasor, exist_ok=True)

        self._csv_yol   = os.path.join(self.klasor, "telemetri.csv")
        self._json_yol  = os.path.join(self.klasor, "oturum.json")
        self._olay_yol  = os.path.join(self.klasor, "olaylar.log")

        self._kayitlar: List[TelemetriKayit] = []
        self._kilit = threading.Lock()
        self._baslangic = time.time()
        self._csv_baslik_yazildi = False

        self._oturum_meta = {
            "baslangic": datetime.now().isoformat(),
            "bitis": None,
            "sure_sn": 0,
            "toplam_kayit": 0,
            "max_irtifa_m": 0.0,
            "max_hiz_ms": 0.0,
            "min_batarya_v": 99.0,
            "olaylar": []
        }

        self.olay_yaz(f"Oturum baslatildi: {self.klasor}")

    # ── Kayıt ekle ─────────────────────────────────────────────

    def kaydet(self, kayit: TelemetriKayit):
        with self._kilit:
            self._kayitlar.append(kayit)
            self._csv_yaz(kayit)
            self._meta_guncelle(kayit)

    def _csv_yaz(self, kayit: TelemetriKayit):
        mod = 'a'
        with open(self._csv_yol, mod, newline='', encoding='utf-8') as f:
            yazar = csv.DictWriter(f, fieldnames=asdict(kayit).keys())
            if not self._csv_baslik_yazildi:
                yazar.writeheader()
                self._csv_baslik_yazildi = True
            yazar.writerow(asdict(kayit))

    def _meta_guncelle(self, kayit: TelemetriKayit):
        if kayit.alt_m > self._oturum_meta["max_irtifa_m"]:
            self._oturum_meta["max_irtifa_m"] = kayit.alt_m
        if kayit.hiz_ms > self._oturum_meta["max_hiz_ms"]:
            self._oturum_meta["max_hiz_ms"] = kayit.hiz_ms
        if kayit.batarya_v < self._oturum_meta["min_batarya_v"]:
            self._oturum_meta["min_batarya_v"] = kayit.batarya_v

    # ── Olay kaydı ─────────────────────────────────────────────

    def olay_yaz(self, mesaj: str):
        zaman = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        satir = f"[{zaman}] {mesaj}"
        print(f"  LOG {satir}")
        with open(self._olay_yol, 'a', encoding='utf-8') as f:
            f.write(satir + "\n")
        self._oturum_meta["olaylar"].append({"zaman": zaman, "mesaj": mesaj})

    # ── Oturumu kapat ──────────────────────────────────────────

    def kapat(self):
        self._oturum_meta["bitis"]         = datetime.now().isoformat()
        self._oturum_meta["sure_sn"]       = round(time.time() - self._baslangic, 1)
        self._oturum_meta["toplam_kayit"]  = len(self._kayitlar)

        with open(self._json_yol, 'w', encoding='utf-8') as f:
            json.dump(self._oturum_meta, f, ensure_ascii=False, indent=2)

        self.olay_yaz("Oturum kapatildi.")
        print(f"\n  Log klasoru: {self.klasor}")

    # ── Son N kaydı getir ──────────────────────────────────────

    def son_kayitlar(self, n: int = 10) -> List[TelemetriKayit]:
        with self._kilit:
            return self._kayitlar[-n:]

    # ── Anlık özet ────────────────────────────────────────────

    def ozet(self) -> dict:
        with self._kilit:
            if not self._kayitlar:
                return {}
            son = self._kayitlar[-1]
            return {
                "sure_sn":      round(time.time() - self._baslangic, 1),
                "kayit_sayisi": len(self._kayitlar),
                "son_konum":    f"{son.lat:.6f}, {son.lon:.6f}",
                "irtifa_m":     son.alt_m,
                "hiz_ms":       son.hiz_ms,
                "batarya":      f"{son.batarya_v:.1f}V ({son.batarya_yuzde}%)",
                "mod":          son.mod,
                "max_irtifa":   self._oturum_meta["max_irtifa_m"],
            }


# ── Mock veri üreticisi ────────────────────────────────────────

class MockTelemetri:
    """Gerçekçi uçuş verisi simüle eder."""

    import random as _r

    def __init__(self):
        self.lat = 37.5730
        self.lon = 36.9180
        self.alt = 0.0
        self.hiz = 0.0
        self.batarya_v = 16.8   # 4S LiPo tam dolu
        self.mod = "GUIDED"
        self.arm = False
        self._t = 0

    def sonraki(self) -> TelemetriKayit:
        import random, math
        self._t += 1

        # Kalkış simülasyonu
        if self._t < 20:
            self.alt   = min(50, self.alt + 2.5)
            self.hiz   = 2.0
            self.arm   = True
        elif self._t < 80:
            self.lat  += random.uniform(-0.00003, 0.00005)
            self.lon  += random.uniform(-0.00003, 0.00005)
            self.hiz   = random.uniform(5, 12)
            self.alt  += random.uniform(-1, 1)
        else:
            self.alt   = max(0, self.alt - 2.0)
            self.hiz   = max(0, self.hiz - 0.5)

        self.batarya_v = max(14.0, self.batarya_v - 0.012)
        yuzde = int((self.batarya_v - 14.0) / (16.8 - 14.0) * 100)

        ts = time.time()
        return TelemetriKayit(
            zaman_damgasi = round(ts, 3),
            zaman_str     = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            lat           = round(self.lat, 7),
            lon           = round(self.lon, 7),
            alt_m         = round(self.alt, 2),
            hiz_ms        = round(self.hiz, 2),
            dikey_hiz_ms  = round(random.uniform(-0.5, 0.5), 2),
            yaw_derece    = round(random.uniform(0, 360), 1),
            pitch_derece  = round(random.uniform(-5, 5), 1),
            roll_derece   = round(random.uniform(-5, 5), 1),
            batarya_v     = round(self.batarya_v, 2),
            batarya_yuzde = max(0, min(100, yuzde)),
            mod           = self.mod,
            arm           = self.arm,
            uydu_sayisi   = random.randint(10, 16),
            hdop          = round(random.uniform(0.9, 1.5), 2),
        )


# ── Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("  KIZIL-2 Telemetri Logger — Mock Test")
    print("=" * 50 + "\n")

    logger = TelemetriLogger()
    mock   = MockTelemetri()

    ADIM = 100

    logger.olay_yaz("Arm komutu gonderildi")

    for i in range(ADIM):
        kayit = mock.sonraki()

        # Özel olayları yakala
        if i == 19:
            logger.olay_yaz(f"Kalkis tamamlandi, irtifa: {kayit.alt_m}m")
        if i == 50:
            logger.olay_yaz("Engel tespit edildi, rota hesaplaniyor")
            kayit.gorev_notu = "engel tespit"
        if i == 80:
            logger.olay_yaz("Inis basladi")

        logger.kaydet(kayit)
        time.sleep(0.02)  # 50Hz simülasyon

    logger.olay_yaz("Disarm")
    logger.kapat()

    print("\nOzet:")
    ozet = logger.ozet()
    for k, v in ozet.items():
        print(f"  {k:<16}: {v}")

    print(f"\nDosyalar:")
    print(f"  telemetri.csv  -> {ADIM} satir")
    print(f"  oturum.json    -> meta veri")
    print(f"  olaylar.log    -> olay gecmisi")
