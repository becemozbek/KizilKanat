"""
RTL-SDR sinyal tarama modülü.
Donanım yokken mock_scan() ile test edilir.
"""
import random
import time

BANTLAR = {
    "gsm_900":  900e6,
    "lte_1800": 1800e6,
    "wifi_2400": 2400e6,
}


def gercek_scan():
    """Gerçek RTL-SDR ile ölçüm — donanım gelince aktif edilir."""
    import rtlsdr
    sdr = rtlsdr.RtlSdr()
    sonuclar = {}
    for ad, frekans in BANTLAR.items():
        sdr.sample_rate = 2.048e6
        sdr.center_freq = frekans
        sdr.gain = "auto"
        ornekler = sdr.read_samples(256 * 1024)
        import numpy as np
        guc = 10 * np.log10(np.mean(np.abs(ornekler) ** 2) + 1e-10)
        sonuclar[ad] = round(guc, 2)
    sdr.close()
    return sonuclar


def mock_scan():
    """Donanım olmadan test verisi üretir."""
    return {
        "gsm_900":   round(random.uniform(-90, -60), 2),
        "lte_1800":  round(random.uniform(-90, -60), 2),
        "wifi_2400": round(random.uniform(-85, -55), 2),
    }


def bolge_skoru(olcum: dict) -> float:
    """
    Ortalama sinyal gücünden normalize skor üretir (0-1).
    Yüksek sinyal = hayatta kalan birey olabilir (afet senaryosu).
    """
    ortalama = sum(olcum.values()) / len(olcum)
    # -90 dBm = 0.0, -50 dBm = 1.0
    skor = (ortalama + 90) / 40
    return max(0.0, min(1.0, round(skor, 3)))


if __name__ == "__main__":
    print("[SDR] Mock tarama başladı...")
    while True:
        olcum = mock_scan()
        skor  = bolge_skoru(olcum)
        print(f"GSM: {olcum['gsm_900']} dBm | "
              f"LTE: {olcum['lte_1800']} dBm | "
              f"WiFi: {olcum['wifi_2400']} dBm | "
              f"Skor: {skor}")
        time.sleep(2)
