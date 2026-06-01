"""
YOLOv8 nesne tespit modülü.
Laptop kamerası veya OAK-D Lite ile çalışır.
"""
import cv2
import time

# Tespit edilecek sınıflar
HEDEF_SINIFLAR = {
    "ambulance":     "mudahale",
    "fire truck":    "mudahale",
    "person":        "insan",
    "car":           "arac",
    "truck":         "arac",
    "debris":        "engel",
    "road":          "gecebilir",
}


class YOLODetector:
    def __init__(self, model_boyut="n", kaynak=0):
        """
        model_boyut: 'n' (nano/hızlı), 's', 'm', 'l', 'x'
        kaynak: 0 = laptop kamerası, video dosyası yolu, veya OAK-D
        """
        from ultralytics import YOLO
        self.model = YOLO(f"yolov8{model_boyut}.pt")
        self.kaynak = kaynak
        print(f"[YOLO] Model yüklendi: yolov8{model_boyut}.pt")

    def kare_isle(self, kare):
        sonuclar = self.model(kare, verbose=False)
        tespitler = []
        for r in sonuclar:
            for box in r.boxes:
                sinif_adi = self.model.names[int(box.cls)]
                tespitler.append({
                    "sinif":     sinif_adi,
                    "tip":       HEDEF_SINIFLAR.get(sinif_adi, "diger"),
                    "guvens":    round(float(box.conf), 3),
                    "bbox":      box.xyxy[0].tolist(),
                    "mudahale":  HEDEF_SINIFLAR.get(sinif_adi) == "mudahale",
                })
        return tespitler

    def canli_tara(self, callback=None):
        cap = cv2.VideoCapture(self.kaynak)
        print("[YOLO] Kamera açıldı, tarama başladı...")
        while True:
            ret, kare = cap.read()
            if not ret:
                break
            tespitler = self.kare_isle(kare)
            if callback:
                callback(tespitler)
            # Görselleştirme
            for t in tespitler:
                x1, y1, x2, y2 = map(int, t["bbox"])
                renk = (0, 0, 255) if t["mudahale"] else (0, 255, 0)
                cv2.rectangle(kare, (x1, y1), (x2, y2), renk, 2)
                cv2.putText(kare, f"{t['sinif']} {t['guvens']}",
                            (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, renk, 1)
            cv2.imshow("KIZIL-2 Nesne Tespiti", kare)
            if cv2.waitKey(1) == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    def tespit_yazdir(tespitler):
        for t in tespitler:
            if t["mudahale"]:
                print(f"[!] MÜDAHALe TESPiT: {t['sinif']} ({t['guvens']})")

    detector = YOLODetector(model_boyut="n", kaynak=0)
    detector.canli_tara(callback=tespit_yazdir)
