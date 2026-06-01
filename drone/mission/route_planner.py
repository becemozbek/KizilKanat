"""
Dinamik görev rota planlayıcısı.
Skorlanmış bölgeleri alır, mesafe + skor ağırlıklı optimal rotayı hesaplar.
"""
import math
from typing import List, Tuple
from drone.mission.scoring_engine import Bolge, oncelik_sirala


def mesafe_hesapla(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki koordinat arası mesafe (metre)."""
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))


def mesafeleri_normalize_et(bolgeler: List[Bolge], drone_lat: float, drone_lon: float) -> List[Bolge]:
    """Drone konumuna olan mesafeleri 0-1 arasına normalize eder."""
    mesafeler = [mesafe_hesapla(drone_lat, drone_lon, b.lat, b.lon) for b in bolgeler]
    max_m = max(mesafeler) if max(mesafeler) > 0 else 1
    for b, m in zip(bolgeler, mesafeler):
        b.mesafe = round(m / max_m, 4)
    return bolgeler


def rota_olustur(bolgeler: List[Bolge], drone_lat: float, drone_lon: float) -> List[Bolge]:
    """
    Greedy nearest-best algoritması:
    Her adımda mevcut konumdan en yüksek skorlu bölgeyi seçer.
    Aktif müdahale olan bölgeler atlanır.
    """
    bolgeler = mesafeleri_normalize_et(bolgeler, drone_lat, drone_lon)
    siralama  = oncelik_sirala(bolgeler)

    rota      = []
    kalan     = [b for b in siralama if not b.aktif_mudahale]
    atlanan   = [b for b in siralama if b.aktif_mudahale]
    mevcut_lat, mevcut_lon = drone_lat, drone_lon

    while kalan:
        # Mevcut konumdan mesafeleri güncelle
        for b in kalan:
            m = mesafe_hesapla(mevcut_lat, mevcut_lon, b.lat, b.lon)
            max_m = max(mesafe_hesapla(mevcut_lat, mevcut_lon, x.lat, x.lon) for x in kalan) or 1
            b.mesafe = round(m / max_m, 4)

        # Skorları yeniden hesapla
        from drone.mission.scoring_engine import skor_hesapla, AGIRLIKLAR
        for b in kalan:
            b.skor = skor_hesapla(b, AGIRLIKLAR)

        # En yüksek skorlu bölgeyi seç
        hedef = max(kalan, key=lambda x: x.skor)
        rota.append(hedef)
        mevcut_lat, mevcut_lon = hedef.lat, hedef.lon
        kalan.remove(hedef)

    return rota, atlanan


def rota_yazdir(rota: List[Bolge], atlanan: List[Bolge], drone_lat: float, drone_lon: float):
    print("\n" + "="*50)
    print("  KIZIL-2 GÖREV ROTASI")
    print("="*50)
    print(f"  Başlangıç: ({drone_lat}, {drone_lon})")
    print("-"*50)

    toplam_mesafe = 0
    onceki_lat, onceki_lon = drone_lat, drone_lon

    for i, b in enumerate(rota, 1):
        m = mesafe_hesapla(onceki_lat, onceki_lon, b.lat, b.lon)
        toplam_mesafe += m
        print(f"  #{i}  {b.id}  ({b.lat}, {b.lon})")
        print(f"       Skor: {b.skor:.3f} | Mesafe: {m:.0f}m")
        onceki_lat, onceki_lon = b.lat, b.lon

    if atlanan:
        print("-"*50)
        print("  ATLANAN BÖLGELER (aktif müdahale):")
        for b in atlanan:
            print(f"  [!] {b.id} — müdahale tespit edildi")

    print("-"*50)
    print(f"  Toplam rota mesafesi: {toplam_mesafe:.0f}m")
    print("="*50)


if __name__ == "__main__":
    from drone.mission.scoring_engine import Bolge

    # Drone başlangıç konumu
    DRONE_LAT = 41.000
    DRONE_LON = 29.000

    bolgeler = [
        Bolge("B1", 41.010, 29.010, arazi_gecebilirlik=0.8, sinyal_yogunluk=0.7,
              engel_yogunluk=0.2, sinyal_anomali=0.1),
        Bolge("B2", 41.005, 29.020, arazi_gecebilirlik=0.5, sinyal_yogunluk=0.9,
              engel_yogunluk=0.4, sinyal_anomali=0.0),
        Bolge("B3", 41.015, 29.005, arazi_gecebilirlik=0.3, sinyal_yogunluk=0.4,
              engel_yogunluk=0.6, sinyal_anomali=0.5, aktif_mudahale=True),
        Bolge("B4", 41.008, 29.015, arazi_gecebilirlik=0.9, sinyal_yogunluk=0.6,
              engel_yogunluk=0.1, sinyal_anomali=0.2),
        Bolge("B5", 41.012, 29.025, arazi_gecebilirlik=0.6, sinyal_yogunluk=0.5,
              engel_yogunluk=0.3, sinyal_anomali=0.3),
    ]

    rota, atlanan = rota_olustur(bolgeler, DRONE_LAT, DRONE_LON)
    rota_yazdir(rota, atlanan, DRONE_LAT, DRONE_LON)
