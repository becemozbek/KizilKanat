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
    """Hazır demo veri — frontend ilk açılışta bunu yükler."""
    from drone.mission.rrt_planner import GPSEngel
    engeller = [
        GPSEngel(41.003, 29.002, 40, "enkaz"),
        GPSEngel(41.005, 29.005, 60, "kirik_yol"),
        GPSEngel(41.007, 29.004, 35, "duvar"),
        GPSEngel(41.006, 29.007, 50, "bina"),
    ]
    rota = kara_rotasi_hesapla(41.0, 29.0, 41.009, 29.009, engeller)
    return {
        "basari": True,
        "rota": [{"lat": r[0], "lon": r[1]} for r in rota],
        "engeller": [{"lat": e.lat, "lon": e.lon,
                      "r": e.yaricap_metre, "tip": e.tip}
                     for e in engeller],
        "baslangic": {"lat": 41.0,   "lon": 29.0},
        "hedef":     {"lat": 41.009, "lon": 29.009},
    }


if __name__ == "__main__":
    uvicorn.run("gcs_server:app", host="0.0.0.0", port=8000, reload=True)
