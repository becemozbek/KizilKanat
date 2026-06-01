"""
Sesli uyarı sistemi — GCS ve saha ekibi için.
pyttsx3 kuruluysa Türkçe TTS, değilse winsound bip sesi.

pip install pyttsx3
"""

import threading
import queue
import time
from enum import Enum
from dataclasses import dataclass

try:
    import pyttsx3
    TTS_MEVCUT = True
except ImportError:
    TTS_MEVCUT = False

try:
    import winsound
    WINSOUND_MEVCUT = True
except ImportError:
    WINSOUND_MEVCUT = False


# ── Uyarı tipleri ──────────────────────────────────────────────

class UyariTip(Enum):
    BILGI    = "bilgi"
    UYARI    = "uyari"
    KRITIK   = "kritik"
    BASARI   = "basari"


@dataclass
class Uyari:
    mesaj: str
    tip: UyariTip = UyariTip.BILGI
    oncelik: int = 5   # 1=en yüksek, 10=en düşük


# ── Hazır uyarı mesajları ──────────────────────────────────────

class Mesaj:
    SISTEM_HAZIR       = Uyari("Sistem hazır",                    UyariTip.BASARI,  8)
    ROTA_HESAPLANDI    = Uyari("Rota hesaplandı, hazır",          UyariTip.BASARI,  7)
    ROTA_BULUNAMADI    = Uyari("Uyarı: Rota bulunamadı",          UyariTip.UYARI,   3)
    ENGEL_TESPIT       = Uyari("Engel tespit edildi",              UyariTip.UYARI,   4)
    HEDEFE_ULASTI      = Uyari("Hedef bölgeye ulaşıldı",          UyariTip.BASARI,  6)
    BAGLANTI_KESILDI   = Uyari("Kritik: Bağlantı kesildi",        UyariTip.KRITIK,  1)
    BATARYA_DUSUK      = Uyari("Uyarı: Batarya düşük",            UyariTip.UYARI,   2)
    BATARYA_KRITIK     = Uyari("Kritik: Batarya kritik, iniyor",  UyariTip.KRITIK,  1)
    ACIL_INIS          = Uyari("Acil iniş başlatıldı",            UyariTip.KRITIK,  1)
    GPS_KAYBI          = Uyari("Uyarı: GPS sinyali kayboldu",     UyariTip.KRITIK,  2)
    KURTARMA_BULUNDU   = Uyari("Kurtarılacak kişi tespit edildi", UyariTip.KRITIK,  1)
    GOREV_TAMAMLANDI   = Uyari("Görev tamamlandı",                UyariTip.BASARI,  6)


# ── Bip kalıpları (winsound fallback) ─────────────────────────

BIP_KALIP = {
    UyariTip.BILGI:   [(800, 100)],
    UyariTip.BASARI:  [(600, 80), (900, 120)],
    UyariTip.UYARI:   [(400, 200), (400, 200)],
    UyariTip.KRITIK:  [(300, 300), (300, 300), (300, 300)],
}


# ── Sesli Uyarı Motoru ─────────────────────────────────────────

class SesliUyari:
    """
    Thread-safe uyarı kuyruğu.
    Kritik uyarılar önce söylenir (öncelik sırası).
    """

    def __init__(self, ses_hizi: int = 160, ses_hacmi: float = 0.9):
        self._kuyruk: queue.PriorityQueue = queue.PriorityQueue()
        self._aktif = False
        self._is_parcasi: threading.Thread = None
        self._ses_hizi = ses_hizi
        self._ses_hacmi = ses_hacmi
        self._tts_motor = None
        self._sessiz = False   # True → ses çıkmaz ama log'a yazar

        if TTS_MEVCUT:
            self._tts_baslat()

    def _tts_baslat(self):
        try:
            self._tts_motor = pyttsx3.init()
            self._tts_motor.setProperty('rate',   self._ses_hizi)
            self._tts_motor.setProperty('volume', self._ses_hacmi)
            # Türkçe ses varsa seç
            sesler = self._tts_motor.getProperty('voices')
            for ses in sesler:
                if 'tr' in ses.id.lower() or 'turkish' in ses.name.lower():
                    self._tts_motor.setProperty('voice', ses.id)
                    break
        except Exception:
            self._tts_motor = None

    def baslat(self):
        """Arka plan uyarı thread'ini başlatır."""
        self._aktif = True
        self._is_parcasi = threading.Thread(target=self._calistir, daemon=True)
        self._is_parcasi.start()

    def durdur(self):
        self._aktif = False

    def sessiz_mod(self, durum: bool):
        """True → sadece print, ses yok."""
        self._sessiz = durum

    def uyar(self, uyari: Uyari):
        """Uyarıyı kuyruğa ekle."""
        self._kuyruk.put((uyari.oncelik, time.time(), uyari))

    def hemen_uyar(self, mesaj: str, tip: UyariTip = UyariTip.BILGI):
        """Tek seferlik uyarı — nesne oluşturmadan."""
        self.uyar(Uyari(mesaj, tip))

    def _calistir(self):
        while self._aktif:
            try:
                _, _, uyari = self._kuyruk.get(timeout=0.5)
                self._oynat(uyari)
            except queue.Empty:
                continue

    def _oynat(self, uyari: Uyari):
        etiket = {
            UyariTip.BILGI:   "[BILGI]",
            UyariTip.BASARI:  "[OK]",
            UyariTip.UYARI:   "[UYARI]",
            UyariTip.KRITIK:  "[KRITIK]",
        }[uyari.tip]
        print(f"  {etiket} {uyari.mesaj}")

        if self._sessiz:
            return

        if self._tts_motor:
            try:
                self._tts_motor.say(uyari.mesaj)
                self._tts_motor.runAndWait()
                return
            except Exception:
                pass

        if WINSOUND_MEVCUT:
            for frekans, sure in BIP_KALIP[uyari.tip]:
                winsound.Beep(frekans, sure)
                time.sleep(0.05)


# ── Singleton erişim ───────────────────────────────────────────

_motor: SesliUyari = None

def motor_al() -> SesliUyari:
    """Uygulama genelinde tek motor örneği döner."""
    global _motor
    if _motor is None:
        _motor = SesliUyari()
        _motor.baslat()
    return _motor


# ── Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 45)
    print("  KIZIL-2 Sesli Uyarı Sistemi — Test")
    print("=" * 45)
    print(f"  TTS mevcut   : {TTS_MEVCUT}")
    print(f"  Winsound     : {WINSOUND_MEVCUT}")
    print()

    motor = SesliUyari()
    motor.baslat()

    senaryolar = [
        Mesaj.SISTEM_HAZIR,
        Mesaj.ENGEL_TESPIT,
        Mesaj.ROTA_HESAPLANDI,
        Mesaj.BATARYA_DUSUK,
        Mesaj.KURTARMA_BULUNDU,   # oncelik=1 — en yüksek
        Mesaj.BAGLANTI_KESILDI,   # oncelik=1
        Mesaj.HEDEFE_ULASTI,
        Mesaj.GOREV_TAMAMLANDI,
    ]

    print("Uyarılar kuyruğa ekleniyor (öncelik sırasıyla çalacak):\n")
    for u in senaryolar:
        motor.uyar(u)
        time.sleep(0.05)

    # Kuyruğun boşalmasını bekle
    time.sleep(3 if not TTS_MEVCUT else 15)
    motor.durdur()
    print("\nTest tamamlandı.")
