"""
Çok kriterli bölge puanlama algoritması.
Score_i = w1*T + w2*S - w3*O - w4*J - w5*D
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class Bolge:
    id: str
    lat: float
    lon: float
    arazi_gecebilirlik: float = 0.0   # T: 0-1
    sinyal_yogunluk:    float = 0.0   # S: 0-1
    engel_yogunluk:     float = 0.0   # O: 0-1
    sinyal_anomali:     float = 0.0   # J: 0-1
    mesafe:             float = 0.0   # D: normalize 0-1
    aktif_mudahale:     bool  = False  # Ambulans/itfaiye tespit
    skor:               float = field(init=False, default=0.0)


AGIRLIKLAR = {
    "w1": 0.30,  # arazi geçebilirliği
    "w2": 0.25,  # sinyal yoğunluğu
    "w3": 0.20,  # engel yoğunluğu
    "w4": 0.15,  # sinyal anomalisi
    "w5": 0.10,  # mesafe
}


def skor_hesapla(bolge: Bolge, agirliklar: dict = AGIRLIKLAR) -> float:
    if bolge.aktif_mudahale:
        return 0.0  # Müdahale edilen bölge sıralamadan düşer

    w = agirliklar
    skor = (
        w["w1"] * bolge.arazi_gecebilirlik
        + w["w2"] * bolge.sinyal_yogunluk
        - w["w3"] * bolge.engel_yogunluk
        - w["w4"] * bolge.sinyal_anomali
        - w["w5"] * bolge.mesafe
    )
    return round(max(0.0, skor), 4)


def oncelik_sirala(bolgeler: List[Bolge]) -> List[Bolge]:
    for b in bolgeler:
        b.skor = skor_hesapla(b)
    return sorted(bolgeler, key=lambda x: x.skor, reverse=True)


if __name__ == "__main__":
    test_bolgeler = [
        Bolge("B1", 41.01, 29.01, arazi_gecebilirlik=0.8, sinyal_yogunluk=0.7,
              engel_yogunluk=0.2, sinyal_anomali=0.1, mesafe=0.3),
        Bolge("B2", 41.02, 29.02, arazi_gecebilirlik=0.5, sinyal_yogunluk=0.9,
              engel_yogunluk=0.4, sinyal_anomali=0.0, mesafe=0.5),
        Bolge("B3", 41.03, 29.03, arazi_gecebilirlik=0.3, sinyal_yogunluk=0.4,
              engel_yogunluk=0.6, sinyal_anomali=0.5, mesafe=0.2, aktif_mudahale=True),
        Bolge("B4", 41.04, 29.04, arazi_gecebilirlik=0.9, sinyal_yogunluk=0.6,
              engel_yogunluk=0.1, sinyal_anomali=0.2, mesafe=0.4),
    ]

    siralama = oncelik_sirala(test_bolgeler)
    print("=== ÖNCELIK SIRALAMASI ===")
    for i, b in enumerate(siralama, 1):
        durum = " [MÜDAHALe VAR - ATLANDI]" if b.aktif_mudahale else ""
        print(f"#{i}  {b.id}  Skor: {b.skor:.4f}{durum}")
