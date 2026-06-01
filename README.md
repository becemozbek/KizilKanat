# KIZIL-2 — KızılKanat

Otonom İHA Tabanlı Çok Kriterli Saha Analizi ve Karar Destekli Görev Rota Planlama Sistemi

**Marmara Üniversitesi — KızılKanat Takımı**

## Klasör Yapısı

```
KızılKanat/
├── drone/
│   ├── mavlink/        # Pixhawk <-> Jetson MAVLink köprüsü
│   ├── vision/         # YOLOv8 nesne tespiti
│   ├── sdr/            # RTL-SDR sinyal tarama
│   ├── slam/           # ORB-SLAM3 entegrasyonu (C++)
│   ├── fusion/         # EKF sensör füzyonu
│   └── mission/        # Görev yöneticisi + skorlama algoritması
├── gcs/
│   ├── backend/        # FastAPI WebSocket sunucu
│   └── frontend/       # Leaflet.js harita arayüzü
├── controller/         # ESP32 kumanda firmware (C++ / PlatformIO)
├── tests/              # Test ve mock veri
└── docs/               # Mimari dokümanlar
```

## Kurulum

```bash
pip install -r requirements.txt
```

## SITL Test (Pixhawk olmadan)

```bash
# 1. WSL'de ArduPilot SITL başlat
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console

# 2. Ayrı terminalde görev yöneticisini başlat
python -m drone.mission.mission_manager
```

## Donanım Bağlantısı (Pixhawk gelince)

`drone/mission/mission_manager.py` dosyasında:
```python
SITL_MODU = False  # Bu satırı değiştir
```
