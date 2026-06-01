"""
Ana görev yöneticisi — tüm modülleri orkestre eder.
SITL modunda Pixhawk olmadan test edilebilir.
"""
import time
import threading
from drone.mavlink.mavlink_bridge import MAVLinkKöprü
from drone.sdr.sdr_scanner import mock_scan, bolge_skoru
from drone.mission.scoring_engine import Bolge, oncelik_sirala

SITL_MODU = True  # Pixhawk gelince False yap


class GorevYoneticisi:
    def __init__(self, bolgeler: list):
        self.mavlink   = MAVLinkKöprü(sitl_modu=SITL_MODU)
        self.bolgeler  = bolgeler
        self.calisiyor = False

    def bolgeleri_guncelle(self):
        """SDR ve YOLO verisiyle bölge skorlarını günceller."""
        for b in self.bolgeler:
            olcum = mock_scan()              # Gerçek SDR gelince gercek_scan()
            b.sinyal_yogunluk = bolge_skoru(olcum)
            # YOLO entegrasyonu buraya eklenecek
        return oncelik_sirala(self.bolgeler)

    def gorevi_baslat(self):
        print("[GÖREV] Başlatılıyor...")
        self.mavlink.mod_değiştir("GUIDED")
        self.calisiyor = True

        while self.calisiyor:
            siralama = self.bolgeleri_guncelle()
            print("\n=== GÜNCEL ÖNCELIK SIRALAMASI ===")
            for i, b in enumerate(siralama, 1):
                print(f"  #{i}  {b.id}  Skor: {b.skor:.3f}"
                      + (" [ATLA]" if b.aktif_mudahale else ""))

            # En yüksek skorlu aktif bölgeye git
            hedef = next((b for b in siralama if not b.aktif_mudahale), None)
            if hedef:
                print(f"[GÖREV] Hedef: {hedef.id} ({hedef.lat}, {hedef.lon})")
                self.mavlink.waypoint_gönder(hedef.lat, hedef.lon, alt=30.0)

            time.sleep(5)

    def gorevi_durdur(self):
        self.calisiyor = False
        self.mavlink.mod_değiştir("RTL")
        print("[GÖREV] Durduruldu, RTL aktif.")


if __name__ == "__main__":
    bolgeler = [
        Bolge("B1", 41.010, 29.010),
        Bolge("B2", 41.012, 29.015),
        Bolge("B3", 41.008, 29.020),
        Bolge("B4", 41.015, 29.012),
    ]
    yonetici = GorevYoneticisi(bolgeler)
    try:
        yonetici.gorevi_baslat()
    except KeyboardInterrupt:
        yonetici.gorevi_durdur()
