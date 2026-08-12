# ecza-robotu

Mecanum tekerlekli, ROS 2 Humble tabanlı gezgin robot. Tüm yığın Raspberry Pi 5
üzerinde Docker container'ları olarak çalışır; motorlar bir ESP32-S3 üzerinde
micro-ROS ile sürülür.

| Bileşen | Donanım / Yazılım |
|---|---|
| Ana bilgisayar | Raspberry Pi 5, Docker Compose, ROS 2 Humble |
| Motor kontrol | ESP32-S3 (micro-ROS, USB CDC `/dev/ttyACM0`) |
| Lidar | RPLidar A2M12 (`/dev/ttyLIDAR`, 256000 baud) |
| IMU | GY-85 (I²C bus 1) |
| Kamera | RPi Camera Module 2 (CSI) |
| Joystick | Logitech F710 (DirectInput modu) |
| Haritalama | slam_toolbox + Nav2, odometri robot_localization EKF |

---

## RViz'i nerede çalıştırmalı?

RViz'in iki çalıştırma yolu var ve **birbirini otomatik dışlamıyorlar** — PC'de
açmak Pi'dekini kapatmaz, `--rviz` ile başlattıysanız ikisi aynı anda çalışır
ve Pi boşuna render etmeye devam eder.

| | **Pi üzerinde** (`--rviz`) | **PC üzerinde** (`--lan`) |
|---|---|---|
| Nasıl bakılır | VNC ile `<pi-ip>:5901` | PC'de doğrudan pencere |
| Render eden | Pi'nin CPU'su (GPU yok, `llvmpipe`) | PC'nin GPU'su |
| Pi'ye maliyeti | **~%30-50 CPU** (720p'de; 1080p'de %120-142 ölçüldü) | **sıfır** |
| DDS profili | varsayılan (izole) | `--lan` şart |
| Yabancı LAN cihazı riski | yok | var (aşağıya bakın) |
| Ne zaman | Hızlı bakış, PC yokken | Uzun süreli izleme, haritalama |

**Karar:** Uzun süre bakacaksanız PC'yi kullanın. Pi'de RViz açık bırakmak
EKF'i aç bırakacak kadar CPU yiyor (`Failed to meet update rate` uyarıları
buradan gelir).

```bash
# Pi'de RViz (Pi render eder)
bash scripts/launch.sh --nav --rviz          # sonra VNC: <pi-ip>:5901

# PC'de RViz (Pi render etmez)  ← önerilen
bash scripts/launch.sh --nav --lan           # Pi'de: --lan var, --rviz YOK
./scripts/rviz_viewer_pc.sh                  # PC'de
```

Pi'de RViz'i sonradan kapatmak için tüm yığını yeniden başlatmaya gerek yok:
```bash
docker compose stop rviz
```
Ama PC'den bakabilmek için DDS profilinin `--lan` olması gerekir; profil
değişikliği container'ların yeniden oluşturulmasını gerektirir.

Ayrıntılar: Pi tarafı için [bölüm 1](#1-robotun-kendi-üzerinde-çalıştırma-pide),
PC tarafı için [bölüm 2.1](#21-rvizi-pcde-açmak-önerilen).

---

## 1. Robotun kendi üzerinde çalıştırma (Pi'de)

Tüm komutlar Pi'de, depo kök dizininde çalıştırılır.

### Temel kullanım

```bash
bash scripts/launch.sh                      # temel yığın (sürüş + kamera + IMU)
bash scripts/launch.sh --lidar              # + RPLidar sürücüsü
bash scripts/launch.sh --nav                # + Nav2 & SLAM (lidar dahil)
bash scripts/launch.sh --nav --map /maps/x.yaml   # kayıtlı haritayla navigasyon
bash scripts/launch.sh --rviz               # + Pi üzerinde RViz (VNC :5901)
bash scripts/launch.sh --nav --lan          # PC'den RViz izlemek için (bkz. bölüm 2)
bash scripts/launch.sh --dev                # geliştirme: kaynak mount + agent debug
```

Bayraklar birleştirilebilir: `bash scripts/launch.sh --nav --rviz`

### Durdurma ve durum

```bash
docker compose down                  # her şeyi durdur
docker compose ps                    # servis durumları
docker compose logs -f teleop        # tek servisin canlı logu
```

> **Not:** `--nav`, `--lidar` ve `--rviz` compose *profile*'ları kullanır. Bu
> yüzden `docker compose up -d --force-recreate` bu servisleri **atlar** —
> profil belirtmeden çalıştırırsanız lidar/Nav2/SLAM eski ayarlarla çalışmaya
> devam eder ve grafikten kopabilir. Elle yeniden başlatırken profili verin:
> ```bash
> COMPOSE_PROFILES=nav,lidar,rviz docker compose up -d --force-recreate
> ```

### Pi üzerinde RViz

`--rviz` ile RViz, container içinde kendi özel Xvfb ekranında çalışır ve
**5901** portunda yayınlanır. O ekranda RViz'den başka hiçbir şey yoktur —
masaüstü, görev çubuğu, ikon görünmez.

| Ne | Nereden |
|---|---|
| Sadece RViz | `<pi-ip>:5901` |
| Tüm Pi masaüstü | `<pi-ip>:5900` (host WayVNC) |

⚠️ Pi'nin kullanılabilir bir GPU'su yok; RViz `llvmpipe` ile her pikseli CPU'da
render eder. 1080p'de tek başına **%120-142 CPU** ölçüldü ve 4 çekirdekte yük
ortalamasını 10-15'e çıkardı; bu EKF'i aç bıraktı ("Failed to meet update
rate"). Bu yüzden çözünürlük 720p'ye sabitlendi. **Uzun süreli izleme için
bölüm 2'deki PC viewer'ı tercih edin** — Pi'ye sıfır yük bindirir.

### Joystick kullanımı (F710, DirectInput)

| Girdi | İşlev |
|---|---|
| Sol sopa | İleri/geri + yanal kayma (strafe) |
| Sağ sopa X | Dönüş |
| LT / RT (6 / 7) | Tek taraf pivot (geniş dönüş) |
| D-pad | Ön/arka aks pivotu |
| **Start (9)** | TELEOP ↔ AUTONOMOUS (Nav2) geçişi |

Buton indeksleri `config/rover_params.yaml` içinde. Bu pad'de fiziksel dizilim
`0-3 = yüz tuşları, 4=LB, 5=RB, 6=LT, 7=RT, 8=Back, 9=Start, 10=L3, 11=R3`
şeklindedir — `jstest`'in bastığı evdev *isimleri* bu diziliminle aynı değildir,
ona göre ayar yapmayın.

**Otonom moddan çıkış:** Start'a basmak TELEOP'a döndürür ve çalışan Nav2
hedefini iptal eder. Hedef bitene kadar Nav2 modu geri kapamaz; Nav2 sustuktan
~3 sn sonra yeni bir hedefle otomatik geçiş yeniden etkinleşir.

### Harita

```bash
bash scripts/reset_map.sh            # SLAM haritasını sıfırla
```
Kayıtlı haritalar `maps/` altına yazılır, `--map` ile geri yüklenir.

---

## 2. PC'den çalıştırma

PC'de bu deponun bir kopyası ve Docker gerekir. PC ile Pi **aynı LAN'da**
olmalıdır. Varsayılan robot adresi scriptlerde `10.42.101.197`; farklıysa
`ROBOT_IP` ortam değişkeniyle verin.

### 2.1 RViz'i PC'de açmak (önerilen)

Pi'nin CPU'sunu tamamen boşaltır — render PC'nin GPU'sunda yapılır.

**Adım 1 — Pi'de**, robotu LAN modunda başlatın (`--lan` var, `--rviz` **yok**):
```bash
bash scripts/launch.sh --nav --lan
```

**Adım 2 — PC'de:**
```bash
./scripts/rviz_viewer_pc.sh              # başlat (ilk seferde imaj derler)
./scripts/rviz_viewer_pc.sh --build      # imajı yeniden derle
./scripts/rviz_viewer_pc.sh --check      # robot DDS'te görünüyor mu test et
./scripts/rviz_viewer_pc.sh --down       # durdur
```

> ### ⚠️ `--lan` olmadan RViz **boş açılır, hata da vermez**
>
> Robot varsayılan olarak `ignoreParticipantFlags=FILTER_DIFFERENT_HOST` ile
> çalışır ve **başka makinelerdeki tüm DDS katılımcılarını yok sayar** — PC
> viewer dahil. Bu filtre, LAN'daki yabancı bir robotu (`vision_bot_urdf` /
> `vision_arm_controller`) domain 0'dan uzak tutmak için var: o cihaz kendi
> `/robot_description`'ını yayınlıyor ve RViz onun URDF'ine takılıp
> `Package [vision_bot_urdf] does not exist` hatalarından başka bir şey
> göstermiyor, ayrıca `/joy`'a üçüncü bir yayıncı ekliyor.
>
> **Takas:** `--lan` açıkken o yabancı cihaz geri gelebilir. PC viewer'ı
> kullanırken açın, işiniz bitince varsayılana dönün.
>
> **Şunları denemeyin:** `interfaceWhiteList` eklemek veya `ROS_DOMAIN_ID`
> değiştirmek — ikisi de ESP32 motor köprüsünü sessizce öldürür ve belirti
> "joystick çalışmıyor" gibi görünür. Ayrıntı: `config/fastdds_udp.xml` ve `.env`.

Boş RViz'de sırayla kontrol edin: (1) Pi `--lan` ile mi başlatıldı,
(2) `ROS_DOMAIN_ID` iki tarafta da 0 mı, (3) Wi-Fi AP multicast'i engelliyor mu
(client isolation). `--check` bu üçünü tek komutta test eder.

### 2.2 PC joystick'iyle uzaktan sürüş

```bash
./scripts/remote_teleop_pc.sh                 # ROBOT_IP'ye bağlan
./scripts/remote_teleop_pc.sh 10.42.101.197   # IP'yi açıkça ver
./scripts/remote_teleop_pc.sh --cam           # kamera akışıyla birlikte
./scripts/remote_teleop_pc.sh --calibrate     # pad eksen/butonlarını eşle
./scripts/remote_teleop_pc.sh --check         # port dinleniyor mu, çık
./scripts/remote_teleop_pc.sh -- --debug --raw # bayrakları doğrudan geçir
```

Gereksinim: `python3`, `pygame` (`pip install pygame`), bir joystick.

**`--lan` gerekmez.** Bu yol düz TCP kullanır (port 9092, satır-sonlu JSON),
DDS değil — hangi profil açık olursa olsun çalışır. Robotun kendi joystick'i
de çalışmaya devam eder; iki girdi birbirine eklenir, `teleop_node` ikisini
ayırt etmez (dead-man, strafe, pivot, scan güvenliği hepsi aynı şekilde geçerli).

### 2.3 Kamera

Kamera iki yoldan yayınlanır:

| Yol | Adres | Notlar |
|---|---|---|
| UDP (önerilen) | `<pi-ip>:8082` | Parçalı JPEG; kayıp paket sadece o kareyi düşürür |
| HTTP (MJPEG) | `http://<pi-ip>:8081/` | Tarayıcıdan bakmak için; Wi-Fi'da TCP takılabilir |

`--cam` ile uzaktan sürüş penceresinde gösterilir. Yayını uzaktan
durdurup başlatmak için:
```bash
./scripts/camera_toggle_pc.sh
```

### 2.4 Haritayı PC'den sıfırlama

```bash
./scripts/reset_map_pc.sh
```

---

## 3. Sık karşılaşılan sorunlar

| Belirti | Sebep / çözüm |
|---|---|
| Joystick çalışmıyor, tekerlekler dönmüyor | ESP32 köprüsünü kontrol edin: `ros2 topic info /wheel_velocities_cmd_f32` **1 abone** göstermeli. 0 ise DDS ayarı köprüyü kesmiştir. |
| RViz PC'de boş | Pi `--lan` ile başlatılmamış (bölüm 2.1) |
| RViz'de yabancı robot modeli / mesh hatası | LAN modundasınız ve yabancı cihaz domain 0'a girmiş — varsayılan profile dönün |
| Harita kayıyor / dağılıyor | Lidar tarama yoğunluğu. `/scan_slam` tarama başına ~190 geçerli nokta vermeli |
| Pi çok yavaş, EKF "Failed to meet update rate" | Pi'de RViz açık olabilir — `docker compose stop rviz`, PC viewer'a geçin |
| PC'de kamera penceresi siyah / görüntü yok | Aşağıdaki kamera teşhisine bakın |

### Kamera görüntüsü PC'ye gelmiyorsa

Sırayla, her adım bir öncekini eler:

```bash
# 1. Pi'de kamera kaynağı üretiyor mu (FPS satırı akmalı)
docker logs -f ecza-robotu-csi_camera-1 | grep FPS

# 2. Host tarafındaki rpicam-vid ayakta mı (container DEĞİL, systemd servisi)
systemctl is-active ecza-robotu-csi-cam

# 3. Robot şu an birine yayın yapıyor mu — 0 ise abone yok demektir
a=$(grep -A1 ^Udp: /proc/net/snmp | tail -1 | awk '{print $4}'); sleep 5
b=$(grep -A1 ^Udp: /proc/net/snmp | tail -1 | awk '{print $4}'); echo $(( (b-a)/5 )) paket/sn

# 4. PC abone olarak kaydolmuş mu
docker logs ecza-robotu-csi_camera-1 | grep "subscriber registered"
```

1-2 çalışıyor ama 3 sıfırsa: PC istemcisinin **video thread'i ölmüştür**.
Joystick TCP bağlantısı ayakta kalabildiği için istemci çalışıyor görünür;
kamera aboneliği ise 5 sn TTL ile sessizce düşer. PC'de istemciyi yeniden
başlatın ve konsoldaki hatayı okuyun:

```bash
./scripts/remote_teleop_pc.sh --cam
```
| Start'a basınca otonom moddan çıkmıyor | Düzeltildi; buton indeksinin `9` olduğunu doğrulayın |

### Değiştirilmemesi gerekenler

- **`ROS_DOMAIN_ID` 0 kalmalı.** Domain, ESP32 firmware'inin kendi
  katılımcısından gelir; host tarafında değiştirmek köprüyü yetim bırakır ve
  ESP32 yeniden flaşlanana kadar tekerlekler ölür.
- **`fastdds_udp.xml`'e `interfaceWhiteList` eklemeyin.** Bu Pi'de `lo`
  arayüzünde MULTICAST bayrağı yok; loopback'e kısıtlamak DDS keşfini öldürür.
  micro-ros-agent seri oturumu kurup "participant created" loglar ama kimse onu
  göremez.
- DDS ile ilgili her değişiklikten sonra `ros2 topic info
  /wheel_velocities_cmd_f32` ile **1 abone** olduğunu doğrulayın.
