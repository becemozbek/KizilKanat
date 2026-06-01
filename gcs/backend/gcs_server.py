"""
GCS Backend — FastAPI + WebSocket
RRT rotasını hesaplar ve frontend'e sunar.
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from typing import List
import uvicorn

from drone.mission.rrt_planner import kara_rotasi_hesapla, GPSEngel

app = FastAPI(title="KIZIL-2 GCS")

# Frontend dosyalarını sun
frontend_path = os.path.join(os.path.dirname(__file__), '../frontend')
app.mount("/static", StaticFiles(directory=frontend_path), name="static")


class EngelGirdi(BaseModel):
    lat: float
    lon: float
    yaricap_metre: float
    tip: str = "enkaz"


class RotaIstegi(BaseModel):
    baslangic_lat: float
    baslangic_lon: float
    hedef_lat: float
    hedef_lon: float
    engeller: List[EngelGirdi]


@app.get("/", response_class=HTMLResponse)
def anasayfa():
    with open(os.path.join(frontend_path, "index.html"), encoding="utf-8") as f:
        return f.read()


@app.post("/api/rota")
def rota_hesapla(istek: RotaIstegi):
    engeller = [
        GPSEngel(e.lat, e.lon, e.yaricap_metre, e.tip)
        for e in istek.engeller
    ]
    rota = kara_rotasi_hesapla(
        istek.baslangic_lat, istek.baslangic_lon,
        istek.hedef_lat,     istek.hedef_lon,
        engeller
    )
    if rota is None:
        return {"basari": False, "mesaj": "Rota bulunamadı"}

    return {
        "basari": True,
        "rota": [{"lat": r[0], "lon": r[1]} for r in rota],
        "engeller": [{"lat": e.lat, "lon": e.lon,
                      "r": e.yaricap_metre, "tip": e.tip}
                     for e in engeller]
    }


@app.get("/api/demo")
def demo_rota():
    """Kahramanmaraş deprem bölgesi — YOLO'dan gelecek engellerin simülasyonu."""
    from drone.mission.rrt_planner import GPSEngel
    # Merkez: 37.5753, 36.9228 — gerçek Kahramanmaraş koordinatları
    # Saha ekibi başlangıç → arama hedefi (~900m mesafe)
    BAS_LAT, BAS_LON = 37.5730, 36.9180
    HIT_LAT, HIT_LON = 37.5820, 36.9290
    engeller = [
        # YOLO "enkaz" tespitleri — çökmüş binalar
        GPSEngel(37.5745, 36.9195, 55, "enkaz"),
        GPSEngel(37.5760, 36.9210, 70, "enkaz"),
        GPSEngel(37.5800, 36.9260, 45, "enkaz"),
        # YOLO "kırık yol" tespitleri
        GPSEngel(37.5755, 36.9230, 40, "kirik_yol"),
        GPSEngel(37.5785, 36.9245, 35, "kirik_yol"),
        # YOLO "duvar" tespitleri — yıkılmış duvarlar
        GPSEngel(37.5770, 36.9200, 30, "duvar"),
        GPSEngel(37.5810, 36.9270, 25, "duvar"),
        # YOLO "bina" tespitleri — hasarlı ama ayakta
        GPSEngel(37.5740, 36.9215, 50, "bina"),
        GPSEngel(37.5795, 36.9255, 40, "bina"),
    ]
    rota = kara_rotasi_hesapla(BAS_LAT, BAS_LON, HIT_LAT, HIT_LON, engeller)
    if rota is None:
        return {"basari": False, "mesaj": "Demo rotası hesaplanamadı"}
    return {
        "basari": True,
        "rota": [{"lat": r[0], "lon": r[1]} for r in rota],
        "engeller": [{"lat": e.lat, "lon": e.lon,
                      "r": e.yaricap_metre, "tip": e.tip}
                     for e in engeller],
        "baslangic": {"lat": BAS_LAT, "lon": BAS_LON},
        "hedef":     {"lat": HIT_LAT, "lon": HIT_LON},
    }


if __name__ == "__main__":
    uvicorn.run("gcs_server:app", host="0.0.0.0", port=8000, reload=True)
