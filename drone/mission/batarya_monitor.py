"""
Batarya izleme modulu.
MAVLink SYS_STATUS mesajindan batarya verisini okur,
eslikleri audio_alert ve telemetri_logger ile konusar.
"""

import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable

from drone.mission.audio_alert import motor_al, Mesaj, Uyari, UyariTip


# ── Batarya durumu ─────────────────────────────────────────────

@dataclass
class BataryaDurum:
    voltaj: float        # Volt
    akim: float          # Amper
    yuzde: int           # %
    kapasite_mah: int    # tüketilen mAh
    sicaklik_c: float    # °C (-1 = bilinmiyor)
    zaman: float


# ── Eşik değerleri ────────────────────────────────────────────

@dataclass
class BataryaEsik:
    uyari_yuzde: int   = 30   # % — sarı uyarı
    kritik_yuzde: int  = 15   # % — kırmızı, RTL başlat
    dusuk_voltaj: float = 14.4  # V  (4S: 3.6V/hücre)
    kritik_voltaj: float = 13.6  # V  (4S: 3.4V/hücre)
    max_sicaklik: float = 60.0   # °C


# ── Batarya Monitör ────────────────────────────────────────────

class BataryaMonitor:
    """
    Sürekli batarya okur, eşik geçilince uyarı verir.
    Pixhawk gelince: mavlink_bridge.batarya_oku() bağlanır.
    """

    def __init__(
        self,
        esik: BataryaEsik = None,
        rtl_geri_donus_cb: Optional[Callable] = None,
        logger=None,
    ):
        self.esik   = esik or BataryaEsik()
        self._ses   = motor_al()
        self._rtl_cb = rtl_geri_donus_cb   # kritik → RTL tetikler
        self._logger = logger

        self._son: Optional[BataryaDurum] = None
        self._aktif = False
        self._is: threading.Thread = None

        # Durum bayrakları — aynı uyarıyı tekrar tekrar vermemek için
        self._uyari_verildi  = False
        self._kritik_verildi = False
        self._rtl_tetiklendi = False

    # ── Başlat / durdur ────────────────────────────────────────

    def baslat(self, okuyucu_fn: Callable, aralik_sn: float = 2.0):
        """
        okuyucu_fn: BataryaDurum dönen fonksiyon
                    (mock ya da mavlink_bridge.batarya_oku)
        """
        self._aktif = True
        self._is = threading.Thread(
            target=self._dongu,
            args=(okuyucu_fn, aralik_sn),
            daemon=True
        )
        self._is.start()

    def durdur(self):
        self._aktif = False

    # ── Ana döngü ──────────────────────────────────────────────

    def _dongu(self, okuyucu_fn, aralik_sn):
        while self._aktif:
            try:
                durum = okuyucu_fn()
                self._son = durum
                self._kontrol_et(durum)
            except Exception as e:
                print(f"  [BATARYA] Okuma hatasi: {e}")
            time.sleep(aralik_sn)

    def _kontrol_et(self, d: BataryaDurum):
        # ── Voltaj kontrolü ───────────────────────────────────
        if d.voltaj <= self.esik.kritik_voltaj and not self._kritik_verildi:
            self._kritik_tetikle(d, f"Kritik voltaj: {d.voltaj:.1f}V")
            return

        if d.voltaj <= self.esik.dusuk_voltaj and not self._uyari_verildi:
            self._uyari_tetikle(d, f"Dusuk voltaj: {d.voltaj:.1f}V")

        # ── Yüzde kontrolü ────────────────────────────────────
        if d.yuzde <= self.esik.kritik_yuzde and not self._kritik_verildi:
            self._kritik_tetikle(d, f"Kritik batarya: %{d.yuzde}")
            return

        if d.yuzde <= self.esik.uyari_yuzde and not self._uyari_verildi:
            self._uyari_tetikle(d, f"Dusuk batarya: %{d.yuzde}")

        # ── Sıcaklık kontrolü ─────────────────────────────────
        if d.sicaklik_c > self.esik.max_sicaklik:
            self._ses.uyar(Uyari(
                f"Batarya sicakligi yuksek: {d.sicaklik_c:.0f}C",
                UyariTip.KRITIK, oncelik=2
            ))
            if self._logger:
                self._logger.olay_yaz(f"BATARYA SICAKLIK: {d.sicaklik_c}C")

        # ── Sağlıklı durum bayrağını sıfırla ─────────────────
        if d.yuzde > self.esik.uyari_yuzde + 5:
            self._uyari_verildi  = False
            self._kritik_verildi = False

    def _uyari_tetikle(self, d, not_str):
        self._uyari_verildi = True
        self._ses.uyar(Mesaj.BATARYA_DUSUK)
        if self._logger:
            self._logger.olay_yaz(f"BATARYA UYARI: {not_str}")
        print(f"  [BATARYA] {not_str}")

    def _kritik_tetikle(self, d, not_str):
        self._kritik_verildi = True
        self._ses.uyar(Mesaj.BATARYA_KRITIK)
        if self._logger:
            self._logger.olay_yaz(f"BATARYA KRITIK: {not_str}")
        print(f"  [BATARYA] {not_str} — RTL tetikleniyor")
        if self._rtl_cb and not self._rtl_tetiklendi:
            self._rtl_tetiklendi = True
            self._rtl_cb()

    # ── Anlık durum ────────────────────────────────────────────

    def durum_al(self) -> Optional[BataryaDurum]:
        return self._son

    def ozet_str(self) -> str:
        if not self._son:
            return "Veri yok"
        d = self._son
        return (f"{d.voltaj:.2f}V | %{d.yuzde} | "
                f"{d.akim:.1f}A | {d.kapasite_mah}mAh tuketildi")


# ── Mock okuyucu ───────────────────────────────────────────────

def mock_batarya_okuyucu(baslangic_v: float = 16.8):
    """Her çağrıda biraz azalan batarya simüle eder."""
    _durum = {"v": baslangic_v, "mah": 0, "t": time.time()}

    def oku() -> BataryaDurum:
        gecen = time.time() - _durum["t"]
        _durum["t"] = time.time()
        _durum["v"] = max(12.0, _durum["v"] - 0.02 * gecen)
        _durum["mah"] += int(15 * gecen)

        v = _durum["v"]
        yuzde = int(max(0, (v - 13.2) / (16.8 - 13.2) * 100))
        return BataryaDurum(
            voltaj=round(v, 2),
            akim=round(12.5 + (16.8 - v) * 0.5, 1),
            yuzde=yuzde,
            kapasite_mah=_durum["mah"],
            sicaklik_c=round(35 + (16.8 - v) * 1.5, 1),
            zaman=time.time(),
        )
    return oku


# ── Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("  KIZIL-2 Batarya Monitor — Mock Test")
    print("=" * 50)
    print("  Esikler: uyari=%30, kritik=%15")
    print("  Voltaj : uyari=14.4V, kritik=13.6V\n")

    def rtl_simulasyon():
        print("\n  >>> RTL komutu gonderildi — drone eve donuyor <<<\n")

    # Hızlı boşalma simülasyonu için düşük voltajdan başla
    okuyucu = mock_batarya_okuyucu(baslangic_v=14.6)
    monitor = BataryaMonitor(rtl_geri_donus_cb=rtl_simulasyon)
    monitor.baslat(okuyucu, aralik_sn=0.5)

    print(f"  {'Sn':>4} | {'Voltaj':>8} | {'Yuzde':>6} | {'Akim':>6} | {'mAh':>6}")
    print("  " + "-" * 45)

    for i in range(20):
        time.sleep(1)
        d = monitor.durum_al()
        if d:
            print(f"  {i+1:>4} | {d.voltaj:>7.2f}V | {d.yuzde:>5}% | "
                  f"{d.akim:>5.1f}A | {d.kapasite_mah:>5}mAh")

    monitor.durdur()
    print("\nTest tamamlandi.")
