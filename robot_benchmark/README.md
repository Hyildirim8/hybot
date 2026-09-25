# robot_benchmark

ecza-robotu için gerçek ortam performans testi ve **akademik rapor üretimi**.

Bu modül mevcut sisteme **dokunmaz**: navigasyon, motor kontrolü ve mevcut
launch/konfigürasyon dosyaları değiştirilmemiştir. Yalnızca çalışan sistemin
topic ve action'larını **dinler**.

---

## ⚠️ GÜVENLİK — önce bunu okuyun

* **Varsayılan mod salt gözlem ve ölçümdür. Bu araç robotu kendiliğinden
  hareket ettirmez.** Hedefleri siz RViz'den verirsiniz, hareketi siz
  yaparsınız; araç yalnızca ölçer.
* Tek istisna `mecanum --allow-motion`'dır. Bu bayrak olmadan mecanum testi
  de yalnızca ölçüm yapar. Bayrak verilirse:
  * ekranda uyarı çıkar ve **`evet` yazmadan** hiçbir şey olmaz,
  * komut `/cmd_vel_nav_smoothed`'e gider, yani `teleop_node`'un AUTO
    çarpışma kapısından (`_auto_scan_safe`) **geçer** — doğrudan motor
    kontrolcüsüne yazılmaz,
  * her yön öncesi lidar ile o yönde boşluk kontrol edilir,
  * firmware teker limiti (18.75 rad/s) aşılacaksa komut reddedilir.
* **ACİL DURDURMA:**
  1. **Ctrl+C** — araç her çıkış yolunda sıfır hız yayınlar.
  2. Joystick'te **Start**'a basıp TELEOP'a geçin.
  3. Gerekirse robotun **güç anahtarını** kapatın.
* Fiziksel güvenlik: testten önce robotun çevresini boşaltın, kablo/eşya
  bırakmayın, tekerleklerin önünü açın. Dar koridor testlerinde robotun
  yanında durun.

---

## Kurulum

Ek kurulum gerekmez; modül depoda hazırdır. İki taraf vardır:

| Taraf | Nerede çalışır | Gereken |
|---|---|---|
| **Ölçüm** (testler) | ROS 2 ortamı (Docker konteyneri) | `rclpy` + ROS mesajları — imajda zaten var |
| **Rapor** (grafik/tablo) | Host (Raspberry Pi) veya PC | `matplotlib` |

`matplotlib` Raspberry Pi OS'ta apt ile kurulur (pip'ten hafif ve çakışmasız):

```bash
sudo apt-get install -y python3-matplotlib
```

> `matplotlib` yoksa **grafikler atlanır**, CSV/JSON/Markdown/LaTeX yine
> üretilir ve bu durum çıktıda açıkça bildirilir.

---

## Hızlı başlangıç

### 1. Gerçek robot olmadan doğrulama (mock)

Robot bağlı değilken tüm rapor zincirini sahte veriyle sınayın:

```bash
cd ~/Workspace/ecza-robotu/robot_benchmark
python3 -m robot_benchmark.cli report --mock
```

Üretilen koşular `meta.mock = True` ile işaretlenir ve raporda
“gerçek robot ölçümü değildir” diye sayılır — sahte veri akademik sonuçlara
sessizce karışamaz.

### 2. Birim testleri

```bash
cd ~/Workspace/ecza-robotu/robot_benchmark
python3 -m unittest discover -s test -v
```

### 3. Gerçek robotta test

Önce robot stack'ini normal şekilde başlatın:

```bash
cd ~/Workspace/ecza-robotu
./scripts/launch.sh --nav -d          # PC'den RViz izleyecekseniz: --nav --lan -d
```

Sonra testi çalıştırın (benchmark profili; normal sürüşte kapalıdır):

```bash
docker compose --profile benchmark run --rm benchmark \
    python3 -m robot_benchmark.cli navigation -e "3. kat koridor" -g "H1-kapi" -r 3
```

Ya da terminal menüsüyle:

```bash
docker compose --profile benchmark run --rm benchmark \
    python3 -m robot_benchmark.cli menu
```

---

## Deneyler ve tam komutlar

Aşağıdaki komutların hepsi şu önekle çalışır:

```bash
cd ~/Workspace/ecza-robotu
docker compose --profile benchmark run --rm benchmark \
    python3 -m robot_benchmark.cli <ALT-KOMUT>
```

### 1. Hedefe ulaşma testi — `navigation`

```bash
... navigation -e "3. kat koridor" -g "H1-kapi" -r 5 --record-bag
```

**Siz ne yapacaksınız:** Araç “RViz'den 2D Nav Goal bekleniyor…” yazınca
RViz'de **2D Nav Goal** ile hedefi verin. Her tekrar için ayrı hedef verin.
Koşu bitince istenirse not girin.

En az 3 farklı hedef için ayrı ayrı çalıştırın (`-g H1-kapi`, `-g H2-asansor`,
`-g H3-oda`), her biri için `-r` ile tekrar sayısı verin.

### 2. Engelden kaçınma ve yeniden planlama — `obstacle`

```bash
... obstacle -e "3. kat koridor" -g "H2-engelli" -r 3
```

**Siz ne yapacaksınız:** Hedefi RViz'den verin, sonra robotun yoluna
**engel koyun** (kutu, sandalye). Koşu bitince araç **“Çarpışma oldu mu?”**
diye sorar — gözlemlediğinizi `e`/`h` ile girin (otomatik çarpışma algılama
donanımı yok).

### 3. Konumlandırma doğruluğu ve tekrarlanabilirlik — `repeatability`

```bash
... repeatability -e "3. kat koridor" -g "H1-kapi" -r 10
```

**Siz ne yapacaksınız:** **Aynı hedefi** her tekrarda RViz'den tekrar verin.
Araç, Nav2 hedef pozu ile robotun `/map` çerçevesindeki son pozu arasındaki
doğrusal (m) ve açısal (derece) hatayı ölçer; rapor ortalama, medyan, std,
min, maks ve %95 güven aralığını üretir.

### 4. Mecanum hareket testi — `mecanum`

Salt ölçüm (önerilen, güvenli):

```bash
... mecanum -e "boş alan" -g "6yon"
```

**Siz ne yapacaksınız:** Araç sırayla her yönü ister
(`ileri, geri, sol, sag, ccw, cw`). Her istendiğinde **joystick ile o hareketi
yapın**; araç tekerleklerin dönmeye başlayıp durmasını algılar ve
başlangıç/bitiş pozundan mesafe, yanal sapma ve açısal sapmayı hesaplar.

Otomatik hareket (dikkat):

```bash
... mecanum -e "boş alan" -g "6yon-oto" --allow-motion --speed 0.12 --duration 1.5
```

Yalnızca `evet` yazdıktan sonra hareket eder. Tek yön denemek için:
`--directions ileri sol`.

### 5. TELEOP/AUTONOMOUS mod geçişi — `mode-switch`

```bash
... mode-switch -e "lab" -g "mod-10tekrar" -r 10
```

**Siz ne yapacaksınız:** Araç her tekrarda “Şimdi Start'a basın … sonra
Enter” der. Joystick'te **Start**'a basın, hemen ardından **Enter**'a basın.
Ölçülen gecikme = mod topic'inin değişmesi − sizin Enter'ınız.

> Bu projede mod geçişini tetikleyen bir servis **yoktur** (çalışan sistemde
> arandı); geçiş joystick veya `/goal_pose` ile olur. Bu yüzden ölçüme
> **insan reaksiyon süresi dahildir** ve raporda böyle belirtilir.

### 6. Haritalama testi — `mapping`

```bash
... mapping -e "3. kat koridor" -g "harita-1" --duration 300
```

**Siz ne yapacaksınız:** Robotu joystick ile veya otonom keşifle (joystick'te
**A**) ortamda gezdirin. Araç harita büyümesini periyodik örnekler.

Aynı ortam için birden fazla koşu yapıp `-e` alanını aynı tutun; rapor
bunları karşılaştırır.

### 7. Kamera akış testi — `camera`

```bash
... camera -e "lab" -g "kamera-1" --duration 60
```

**Siz ne yapacaksınız:** Bir şey yapmanıza gerek yok, tamamen pasif.

### 8. Çalışma süresi ve batarya — `battery`

```bash
... battery -e "3. kat koridor" -g "dayanim-1" --duration 3600 --unit V
```

**Siz ne yapacaksınız:** Araç başlangıçta ve bitişte **batarya değerini
sorar** — voltmetre/şarj göstergesinden okuyup girin. Bilmiyorsanız boş
bırakın (`N/A` yazılır, uydurulmaz).

> **Bu robotta batarya ölçüm donanımı ve topic'i yoktur** (tüm kod tabanı
> `battery|voltage|vbat` için arandı). Bu yüzden değerler operatörden alınır
> ve **kalan çalışma süresi tahmin edilmez** (kapasite bilinmiyor).

### 9. Rapor üretimi — `report`

Grafikler için **host'ta** çalıştırın (matplotlib orada kurulu):

```bash
cd ~/Workspace/ecza-robotu/robot_benchmark
python3 -m robot_benchmark.cli report
```

---

## Çıktılar

### Koşu başına — `robot_benchmark/results/<experiment_id>/`

| Dosya | İçerik |
|---|---|
| `meta.json` | koşu kimliği, tarih, ortam, hedef, tekrar, operatör notu, `partial`, `mock` |
| `metrics.json` | ölçülen metrikler (ölçülemeyen → `null`) |
| `events.jsonl` | zaman damgalı olaylar (anlık diske akıtılır) |
| `samples/*.csv` | zaman serileri (poz, kamera kareleri, harita büyümesi…) |
| `run.csv` | tek satırlık düz özet |
| `bag/` | `--record-bag` verildiyse rosbag2 kaydı |

### Toplu — `robot_sonuc/benchmark_outputs/`

| Dosya | İçerik |
|---|---|
| `summary.json` | tüm koşuların özeti, deney ve hedef bazlı istatistikler |
| `summary_runs.csv` | her koşu bir satır (ham, hiçbir koşu atılmaz) |
| `summary_metrics.csv` | metrik × (deney, ortam, hedef) özet tablosu |
| `sonuclar.md` | Markdown tablolar (rapora yapıştırılabilir) |
| `sonuclar.tex` | LaTeX tablolar (`booktabs`; `\input` ile alınır) |
| `01_basari_orani.png` | başarı oranı |
| `02_hedefe_ulasma_suresi.png` | hedefe ulaşma süresi (kutu grafiği) |
| `03_planlanan_vs_gercek_mesafe.png` | planlanan / gerçek mesafe |
| `04_konum_hatasi_kutu.png` | doğrusal + açısal konum hatası |
| `05_yeniden_planlama.png` | yeniden planlama sayısı |
| `06_harita_kapsama.png` | harita kapsama oranı |
| `07_kamera_fps_aralik.png` | kamera FPS ve kare aralığı |

Grafikler 200 dpi, Türkçe başlık/eksen/birim ve **örnek sayısı (n)** içerir.

---

## Veri dürüstlüğü kuralları

Bu üç kural koda gömülüdür ve birim testleriyle korunur:

1. **Eksik metrik `0` ile doldurulmaz.** Ölçülemeyen her şey `null`/`N/A`'dır.
   `0` geçerli bir ölçümdür; ölçüm yokluğuyla karıştırılmaz.
2. **Başarısız koşular silinmez.** `succeeded` / `aborted` / `canceled` ayrı
   sayılır ve rapora girer.
3. **Harita kalitesi tek bir uydurma puana indirgenmez.** Ölçülebilen her
   metrik ayrı raporlanır.

Ayrıca:
* Tek ölçümden standart sapma veya güven aralığı **üretilmez**.
* Güven aralığı küçük örneklemde **Student-t** ile hesaplanır (n<30'da 1.96
  kullanmak aralığı olduğundan dar gösterir).
* Kamera uçtan uca gecikmesi yalnızca mesaj damgası güvenilirse ölçülür;
  ölçülse bile “ROS yayın→alım gecikmesi” olarak etiketlenir, sensörden
  ekrana gecikme olduğu **iddia edilmez**.

---

## Test yarıda kesilirse

`Ctrl+C` güvenlidir: o ana kadarki bütün veri diske yazılmıştır
(`events.jsonl` ve `samples/*.csv` anlık akıtılır) ve koşu `meta.partial =
true` ile işaretlenir. Kısmi koşular rapora dahil edilir ve ayrıca sayılır.

---

## Kullanılan ROS arayüzleri

Hepsi çalışan sistemden doğrulanmıştır; tek kaynak
[`robot_benchmark/ros_names.py`](robot_benchmark/ros_names.py). İsim
değişirse yalnızca o dosya güncellenir.

**Topic'ler**

| Topic | Tip | Kullanım |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | gidilen mesafe (pürüzsüz odom) |
| `/odom` | `nav_msgs/Odometry` | ham tekerlek odometrisi |
| `/map` | `nav_msgs/OccupancyGrid` | harita metrikleri |
| `/plan` | `nav_msgs/Path` | planlanan yol, yeniden planlama |
| `/local_plan` | `nav_msgs/Path` | yerel yörünge |
| `/scan` | `sensor_msgs/LaserScan` | engel algılama, boşluk kontrolü |
| `/autonomous_mode` | `std_msgs/Bool` (latched) | mod doğrulaması |
| `/slam_manager/status` | `std_msgs/String` | keşif durumu |
| `/camera_csi/image_raw/compressed` | `sensor_msgs/CompressedImage` | kamera FPS |
| `/wheel_velocities` | `std_msgs/Float32MultiArray` | **duruş tespiti** |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz hedefi |
| `/behavior_tree_log` | `nav2_msgs/BehaviorTreeLog` | recovery / planlama |
| `/imu/data_raw` | `sensor_msgs/Imu` | bag kaydı |

**Action'lar** (yalnızca durum dinlenir, **hedef gönderilmez**)

| Action | Kullanım |
|---|---|
| `/navigate_to_pose` | `_action/status` → succeeded/aborted/canceled ayrımı |
| | `_action/feedback` → `navigation_time`, `number_of_recoveries`, `distance_remaining` |

**Frame'ler**

| Frame | Not |
|---|---|
| `map` | hedef ve konum hatası bu çerçevede ölçülür |
| `odom` | gidilen mesafe bu çerçeveden entegre edilir (pürüzsüz) |
| `base_link` | robot gövdesi |
| `laser_frame` | lidar; gövdeye göre yaw = 180° |

> **Neden iki ayrı çerçeve?** `slam_toolbox`, `map→odom` düzeltmesini
> süreksiz uygular. Mesafeyi `map` çerçevesinde entegre etmek bu sıçramaları
> “gidilen yol” sayar ve mesafeyi şişirir. Bu yüzden **mesafe `odom`'dan**,
> **hata `map`'ten** ölçülür.

---

## Otomatik ölçülen ve operatör girişi gereken metrikler

| Metrik | Kaynak |
|---|---|
| Hedef durumu (succeeded/aborted/canceled) | **Otomatik** — `/navigate_to_pose/_action/status` |
| Süre, planlanan yol, gidilen mesafe, ortalama hız | **Otomatik** |
| Doğrusal / açısal konum hatası | **Otomatik** — hedef pozu vs `map→base_link` |
| Yeniden planlama sayısı, ilk yeniden planlama süresi | **Otomatik** — `/plan` içerik değişimi |
| Nav2 recovery sayısı | **Otomatik** — action feedback |
| Engel algılama olayı | **Otomatik** — `/scan` eşik geçişi |
| Harita metrikleri (çözünürlük, alan, hücreler, kapsama) | **Otomatik** — `/map` |
| Kamera FPS, kare sayısı, kesinti, çözünürlük, aralık dağılımı | **Otomatik** |
| Mecanum mesafe / yanal sapma / açısal sapma | **Otomatik** (hareketi siz yaparsınız) |
| Mod geçiş doğrulaması | **Otomatik** — `/autonomous_mode` |
| **Çarpışma** | **OPERATÖR** — donanım yok, testte sorulur |
| **Mod geçiş isteği anı** | **OPERATÖR** — Enter ile işaretlenir |
| **Batarya değerleri** | **OPERATÖR** — batarya topic'i yok |
| **Ortam adı, hedef adı, notlar** | **OPERATÖR** |

---

## Sorun giderme

**`rclpy bulunamadı`** — ROS ortamını kaynaklamadınız. Konteyner içinde
çalıştırın veya `source /opt/ros/humble/setup.bash`.

**`/autonomous_mode okunamadı`** — `teleop` konteyneri çalışmıyor olabilir:
`docker compose --profile nav ps`.

**Hedef gelmiyor** — RViz'in robota bağlı olduğundan emin olun. PC'den
bağlanıyorsanız robot `--lan` ile başlatılmalı (bkz. ana README bölüm 2).

**Grafikler atlanıyor** — `matplotlib` kurulu değil. Host'ta
`sudo apt-get install -y python3-matplotlib`, sonra raporu host'ta üretin.

**Konteyner sonuç yazamıyor** — `robot_benchmark/results` ve
`robot_sonuc/benchmark_outputs` dizinlerinin yazılabilir olduğundan emin olun.
