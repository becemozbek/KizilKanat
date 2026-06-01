"""
RRT (Rapidly-exploring Random Tree) tabanlı kara rotası planlayıcı.
YOLOv8'den gelen engel koordinatlarını kullanarak saha ekibi için
güvenli kara rotası hesaplar.

Koordinat sistemi: GPS (lat, lon) — metre cinsinden çalışmak için
Haversine ile metreye çevrilir, RRT metre uzayında çalışır,
sonuç tekrar GPS'e çevrilir.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ── Veri yapıları ──────────────────────────────────────────────────

@dataclass
class Nokta:
    x: float  # metre (doğu)
    y: float  # metre (kuzey)

    def mesafe(self, diger: 'Nokta') -> float:
        return math.sqrt((self.x - diger.x)**2 + (self.y - diger.y)**2)


@dataclass
class Engel:
    merkez: Nokta
    yaricap: float  # metre — engelin etki alanı


@dataclass
class RRTDugum:
    nokta: Nokta
    ebeveyn: Optional['RRTDugum'] = None


# ── Koordinat dönüşümleri ─────────────────────────────────────────

def gps_to_metre(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Nokta:
    """GPS koordinatını referans noktaya göre metreye çevirir."""
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * 6371000
    y = math.radians(lat - ref_lat) * 6371000
    return Nokta(x, y)


def metre_to_gps(nokta: Nokta, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Metre koordinatını GPS'e çevirir."""
    lat = ref_lat + math.degrees(nokta.y / 6371000)
    lon = ref_lon + math.degrees(nokta.x / (6371000 * math.cos(math.radians(ref_lat))))
    return round(lat, 7), round(lon, 7)


# ── RRT Algoritması ───────────────────────────────────────────────

class RRTPlanlayici:
    def __init__(
        self,
        baslangic: Nokta,
        hedef: Nokta,
        engeller: List[Engel],
        alan_genislik: float = 1000,   # metre
        alan_yukseklik: float = 1000,  # metre
        adim_uzunluk: float = 10,      # metre — her RRT adımı
        max_iterasyon: int = 5000,
        hedef_tolerans: float = 15,    # metre — hedefe bu kadar yaklaşınca tamam
        guvenlik_payi: float = 5,      # metre — engel yarıçapına eklenir
    ):
        self.baslangic       = baslangic
        self.hedef           = hedef
        self.engeller        = engeller
        self.alan_genislik   = alan_genislik
        self.alan_yukseklik  = alan_yukseklik
        self.adim_uzunluk    = adim_uzunluk
        self.max_iterasyon   = max_iterasyon
        self.hedef_tolerans  = hedef_tolerans
        self.guvenlik_payi   = guvenlik_payi
        self.agac: List[RRTDugum] = []

    def _rastgele_nokta(self) -> Nokta:
        """Alanda rastgele nokta üretir, %15 ihtimalle hedefe bias yapar."""
        if random.random() < 0.15:
            return self.hedef
        return Nokta(
            random.uniform(0, self.alan_genislik),
            random.uniform(0, self.alan_yukseklik)
        )

    def _en_yakin_dugum(self, nokta: Nokta) -> RRTDugum:
        return min(self.agac, key=lambda d: d.nokta.mesafe(nokta))

    def _yeni_nokta(self, en_yakin: Nokta, rastgele: Nokta) -> Nokta:
        """en_yakin'dan rastgele yönünde adim_uzunluk kadar ilerler."""
        mesafe = en_yakin.mesafe(rastgele)
        if mesafe < self.adim_uzunluk:
            return rastgele
        oran = self.adim_uzunluk / mesafe
        return Nokta(
            en_yakin.x + oran * (rastgele.x - en_yakin.x),
            en_yakin.y + oran * (rastgele.y - en_yakin.y)
        )

    def _engel_var_mi(self, a: Nokta, b: Nokta) -> bool:
        """a-b doğru parçasının herhangi bir engelle kesişip kesişmediğini kontrol eder."""
        for engel in self.engeller:
            r = engel.yaricap + self.guvenlik_payi
            # Nokta-doğru mesafesi hesabı
            dx, dy = b.x - a.x, b.y - a.y
            uzunluk = math.sqrt(dx**2 + dy**2)
            if uzunluk == 0:
                continue
            t = max(0, min(1, ((engel.merkez.x - a.x) * dx +
                               (engel.merkez.y - a.y) * dy) / (uzunluk**2)))
            en_yakin_x = a.x + t * dx
            en_yakin_y = a.y + t * dy
            mesafe = math.sqrt((engel.merkez.x - en_yakin_x)**2 +
                               (engel.merkez.y - en_yakin_y)**2)
            if mesafe < r:
                return True
        return False

    def _yolu_duzelt(self, yol: List[Nokta]) -> List[Nokta]:
        """
        Yolu düzeltir: gereksiz ara noktaları kaldırır.
        İki nokta arasında engel yoksa aralarındaki noktaları atlar.
        """
        if len(yol) <= 2:
            return yol
        duzeltilmis = [yol[0]]
        i = 0
        while i < len(yol) - 1:
            j = len(yol) - 1
            while j > i + 1:
                if not self._engel_var_mi(yol[i], yol[j]):
                    break
                j -= 1
            duzeltilmis.append(yol[j])
            i = j
        return duzeltilmis

    def planla(self) -> Optional[List[Nokta]]:
        """RRT çalıştırır, yol bulursa düzeltilmiş nokta listesi döner."""
        self.agac = [RRTDugum(self.baslangic)]

        for _ in range(self.max_iterasyon):
            rastgele   = self._rastgele_nokta()
            en_yakin_d = self._en_yakin_dugum(rastgele)
            yeni       = self._yeni_nokta(en_yakin_d.nokta, rastgele)

            if self._engel_var_mi(en_yakin_d.nokta, yeni):
                continue

            yeni_dugum = RRTDugum(yeni, ebeveyn=en_yakin_d)
            self.agac.append(yeni_dugum)

            if yeni.mesafe(self.hedef) <= self.hedef_tolerans:
                # Hedefe ulaşıldı — yolu geri izle
                yol = [self.hedef]
                mevcut = yeni_dugum
                while mevcut:
                    yol.append(mevcut.nokta)
                    mevcut = mevcut.ebeveyn
                yol.reverse()
                return self._yolu_duzelt(yol)

        return None  # Yol bulunamadı


# ── GPS tabanlı arayüz ────────────────────────────────────────────

@dataclass
class GPSEngel:
    lat: float
    lon: float
    yaricap_metre: float
    tip: str = "enkaz"  # enkaz / duvar / kirik_yol / bina


def kara_rotasi_hesapla(
    baslangic_lat: float, baslangic_lon: float,
    hedef_lat: float,     hedef_lon: float,
    gps_engeller: List[GPSEngel],
    alan_m: float = 1000,
) -> Optional[List[Tuple[float, float]]]:
    """
    GPS koordinatlarıyla çalışan ana fonksiyon.
    Döner: GPS koordinat listesi [(lat, lon), ...] veya None
    """
    ref_lat = (baslangic_lat + hedef_lat) / 2
    ref_lon = (baslangic_lon + hedef_lon) / 2

    bas  = gps_to_metre(baslangic_lat, baslangic_lon, ref_lat, ref_lon)
    hit  = gps_to_metre(hedef_lat,     hedef_lon,     ref_lat, ref_lon)

    # Koordinatları pozitife taşı
    offset_x = min(bas.x, hit.x) - 50
    offset_y = min(bas.y, hit.y) - 50
    bas = Nokta(bas.x - offset_x, bas.y - offset_y)
    hit = Nokta(hit.x - offset_x, hit.y - offset_y)

    engeller = []
    for e in gps_engeller:
        m = gps_to_metre(e.lat, e.lon, ref_lat, ref_lon)
        engeller.append(Engel(Nokta(m.x - offset_x, m.y - offset_y), e.yaricap_metre))

    rrt  = RRTPlanlayici(bas, hit, engeller, alan_m, alan_m)
    yol  = rrt.planla()

    if yol is None:
        return None

    return [metre_to_gps(Nokta(n.x + offset_x, n.y + offset_y), ref_lat, ref_lon)
            for n in yol]


# ── Test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Saha ekibi konumu → hedef bölge
    BASLANGIC_LAT, BASLANGIC_LON = 41.0000, 29.0000
    HEDEF_LAT,     HEDEF_LON     = 41.0090, 29.0090

    # YOLOv8'den gelen mock engeller
    engeller = [
        GPSEngel(41.003, 29.002, yaricap_metre=40, tip="enkaz"),
        GPSEngel(41.005, 29.005, yaricap_metre=60, tip="kirik_yol"),
        GPSEngel(41.007, 29.004, yaricap_metre=35, tip="duvar"),
        GPSEngel(41.006, 29.007, yaricap_metre=50, tip="bina"),
    ]

    print("RRT Kara Rotası Hesaplanıyor...")
    print(f"Başlangıç : ({BASLANGIC_LAT}, {BASLANGIC_LON})")
    print(f"Hedef     : ({HEDEF_LAT}, {HEDEF_LON})")
    print(f"Engel sayısı: {len(engeller)}")
    print()

    rota = kara_rotasi_hesapla(
        BASLANGIC_LAT, BASLANGIC_LON,
        HEDEF_LAT,     HEDEF_LON,
        engeller
    )

    if rota:
        print(f"Rota bulundu! {len(rota)} waypoint:")
        toplam = 0
        for i, (lat, lon) in enumerate(rota):
            if i > 0:
                prev = rota[i-1]
                d = math.sqrt(
                    (math.radians(lat - prev[0]) * 6371000)**2 +
                    (math.radians(lon - prev[1]) * math.cos(math.radians(lat)) * 6371000)**2
                )
                toplam += d
                print(f"  #{i+1}  ({lat}, {lon})  +{d:.0f}m")
            else:
                print(f"  #{i+1}  ({lat}, {lon})  [başlangıç]")
        print(f"\nToplam kara rotası: {toplam:.0f}m")
        print("\nGCS'e gönderilecek JSON:")
        import json
        print(json.dumps({"rota": [{"lat": r[0], "lon": r[1]} for r in rota],
                          "engeller": [{"lat": e.lat, "lon": e.lon,
                                        "r": e.yaricap_metre, "tip": e.tip}
                                       for e in engeller]}, indent=2))
    else:
        print("HATA: Rota bulunamadı — engeller çok yoğun veya alan çok dar.")
