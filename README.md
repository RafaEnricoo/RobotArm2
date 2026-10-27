# Robot Arm + Motor DC (ESP32)

Kontrol robot arm 4 servo (Base, Siku, Lengan Atas, Gripper) + motor DC dari browser laptop, dengan tampilan 3D, mode manual, dan perekam program gerakan. Motor hanya dikontrol arah kiri/kanan dan berhenti; ENA L298N memakai jumper sehingga kecepatannya tidak diatur dari web.

Ada **dua versi** yang bisa dipilih:

| Versi | Folder | Koneksi | Butuh Python? |
|---|---|---|---|
| **USB** | `RobotArmUSB/` + `RobotArmControl.html` | Kabel USB (Web Serial) | Tidak |
| **WiFi** | `RobotArmWiFi/` | WiFi (WebSocket lewat server laptop) | Ya |

---

## Struktur Folder

```
RobotArm2/
├── RobotArmControl.html      # Web controller versi USB (buka langsung di Chrome/Edge)
├── RobotArmUSB/
│   └── RobotArmUSB.ino       # Sketch ESP32 versi USB
└── RobotArmWiFi/
    ├── RobotArmWiFi.ino      # Sketch ESP32 versi WiFi
    ├── RobotArmControl.html  # Web controller versi WiFi (disajikan oleh server.py)
    ├── server.py             # Server laptop: jembatan browser <-> ESP32
    ├── requirements.txt      # Dependensi Python (aiohttp)
    ├── run.sh                # Jalankan server (Linux / macOS)
    └── run.bat               # Jalankan server (Windows)
```

---

## Hardware

| Komponen | Pin ESP32 |
|---|---|
| Servo Base | GPIO 13 |
| Servo Siku | GPIO 14 |
| Servo Lengan Atas | GPIO 27 |
| Servo Gripper | GPIO 26 |
| L298N ENA (WiFi) | Jumper tetap terpasang |
| L298N IN1 (WiFi) | GPIO 16 |
| L298N IN2 (WiFi) | GPIO 17 |

- Board: **ESP32 Dev Module**
- Servo: SG90 (pulsa 500–2400 µs)
- L298N versi WiFi: pasang jumper ENA; satukan GND L298N dengan GND ESP32 dan gunakan catu daya motor yang sesuai. Web menyediakan arah kiri/kanan dan berhenti tanpa kontrol kecepatan.
- Kalau servo masih ambil daya dari USB laptop, biarkan `LOW_POWER_MODE = true` di sketch supaya ESP tidak restart (brownout). Kalau servo sudah pakai supply 5V terpisah, boleh diubah ke `false`.
- Kalibrasi batas sudut servo (`SERVO_MIN`, `SERVO_MAX`, `SERVO_HOME`) di bagian **KONFIGURASI** sketch sesuai rangka kamu, supaya servo tidak memaksa mentok.

---

## Persiapan Arduino IDE (untuk kedua versi)

1. Install **Arduino IDE 2.x**.
2. Tambahkan board ESP32: *File → Preferences → Additional boards manager URLs*, isi:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
   lalu *Tools → Board → Boards Manager*, cari **esp32** (by Espressif), install.
3. Install library di *Library Manager*:
   - **ESP32Servo** (Kevin Harrington)
   - **ArduinoJson** versi 7.x (Benoit Blanchon)
   - **WebSockets** (Markus Sattler), khusus versi WiFi
4. Pilih *Tools → Board → ESP32 Dev Module* dan port COM / `/dev/ttyUSB0` ESP kamu.

---

## Versi USB (tanpa WiFi)

1. Buka `RobotArmUSB/RobotArmUSB.ino` di Arduino IDE, lalu **Upload**.
2. **Tutup Serial Monitor** Arduino IDE (port harus bebas).
3. Buka `RobotArmControl.html` (yang di folder utama) pakai **Google Chrome** atau **Microsoft Edge**. Firefox tidak mendukung Web Serial.
4. Klik **Hubungkan ke ESP**, lalu pilih port ESP kamu.

Komunikasi: JSON per baris lewat Serial **115200 baud**.

> Linux: kalau port tidak bisa dibuka, tambahkan user ke grup `uucp` (Arch) atau `dialout` (Ubuntu/Debian):
> `sudo usermod -aG uucp $USER`, lalu logout & login lagi.

---

## Versi WiFi

```
Browser  <--WebSocket-->  server.py (laptop)  <--WebSocket/WiFi-->  ESP32
```

### 1. Atur sketch
Buka `RobotArmWiFi/RobotArmWiFi.ino`, lalu isi bagian **KONFIGURASI**:

```cpp
const char* WIFI_SSID = "NAMA_WIFI";      // nama WiFi
const char* WIFI_PASS = "PASSWORD_WIFI";  // password WiFi
const char* SERVER_HOST = "";             // kosongkan = ESP cari laptop otomatis
```

- Laptop dan ESP32 **harus di WiFi yang sama**. ESP32 cuma bisa WiFi **2.4 GHz**.
- `SERVER_HOST` kosong artinya ESP mencari laptop otomatis lewat broadcast UDP (port 8766).
- Kalau auto-cari gagal (WiFi kampus sering memblokir broadcast), isi `SERVER_HOST` dengan IP laptop yang tampil saat `server.py` dijalankan, misalnya `"192.168.1.10"`.

> Jangan commit password WiFi asli ke repo ini (repo-nya publik).

Lalu **Upload** ke ESP32.

### 2. Jalankan server di laptop
Butuh **Python 3.9+**. Venv dan dependensi dibuat **otomatis** saat pertama kali dijalankan.

**Linux / macOS**
```bash
cd RobotArmWiFi
./run.sh
```

**Windows**: klik dua kali `RobotArmWiFi\run.bat`, atau lewat CMD:
```bat
cd RobotArmWiFi
run.bat
```

<details>
<summary>Cara manual (tanpa run.sh / run.bat)</summary>

```bash
cd RobotArmWiFi
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python server.py                 # opsi: --port 8765
```
</details>

Server akan menampilkan:
```
 Buka di browser : http://localhost:8765
 IP laptop       : 192.168.x.x  (isi SERVER_HOST di sketch kalau auto-cari gagal)
```

### 3. Buka controller
Buka **http://localhost:8765** di browser laptop. Status ESP akan berubah jadi terhubung begitu ESP32 tersambung ke server.

### Fitur keamanan
- Server mengirim *ping* ke ESP tiap 1 detik selama ada tab browser terbuka.
- Kalau ping berhenti (browser ditutup / laptop putus), **ESP menghentikan conveyor sendiri setelah 3 detik**.

### Port yang dipakai
| Port | Protokol | Fungsi |
|---|---|---|
| 8765 | TCP (HTTP + WebSocket) | Web controller & koneksi ESP |
| 8766 | UDP broadcast | Auto-discovery IP laptop |

Kalau ESP tidak bisa konek, izinkan port ini di firewall laptop.

---

## Troubleshooting

| Masalah | Solusi |
|---|---|
| ESP restart terus saat servo bergerak | Pakai supply 5V terpisah untuk servo, atau biarkan `LOW_POWER_MODE = true` |
| Tombol "Hubungkan ke ESP" tidak muncul port | Tutup Serial Monitor, pakai Chrome/Edge, cek kabel USB (harus kabel data) |
| Versi WiFi: ESP tidak terhubung ke server | Cek SSID/password, pastikan WiFi 2.4 GHz & satu jaringan, isi `SERVER_HOST` manual, cek firewall port 8765/8766 |
| Arah motor WiFi terasa terbalik | Ubah `CONVEYOR_INVERT = true` di sketch WiFi |
| Log ESP | Buka Serial Monitor 115200 baud (versi WiFi) |
