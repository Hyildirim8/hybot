# Robot Performans Testi Sonuçları

- Toplam koşu sayısı: **20**
- Yarıda kesilen koşu: **0** (veriler korundu, sonuçlara dahil)
- Sahte/mock koşu: **0** (gerçek robot ölçümü değildir)

> Eksik ölçümler `N/A` ile gösterilir; 0 ile doldurulmaz. Başarısız koşular sonuçlardan çıkarılmaz.


## Deney: camera

- Koşu sayısı: **1**
- Hedefe ulaşma oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Kamera FPS | FPS | 1 | 14.996 | 14.996 | N/A | 14.996 | 14.996 | N/A |
| Kareler arası süre | s | 1 | 0.067 | 0.067 | N/A | 0.067 | 0.067 | N/A |
| Toplam kesinti | s | 1 | 0.000 | 0.000 | N/A | 0.000 | 0.000 | N/A |
| Alınan kare sayısı | - | 1 | 900.000 | 900.000 | N/A | 900.000 | 900.000 | N/A |


## Deney: mapping

- Koşu sayısı: **1**
- Hedefe ulaşma oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Harita kapsama oranı | oran | 1 | 0.372 | 0.372 | N/A | 0.372 | 0.372 | N/A |
| Bilinen harita alanı | m² | 1 | 527.450 | 527.450 | N/A | 527.450 | 527.450 | N/A |
| Dolu hücre sayısı | - | 1 | 20170.000 | 20170.000 | N/A | 20170.000 | 20170.000 | N/A |
| Haritalama süresi | s | 1 | 40.784 | 40.784 | N/A | 40.784 | 40.784 | N/A |


## Deney: mecanum

- Koşu sayısı: **6**
- Hedefe ulaşma oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 6 | 0.475 | 0.636 | 0.364 | 0.013 | 0.836 | [0.094; 0.857] |
| Yanal/ikincil sapma | m/° | 6 | 0.021 | 0.018 | 0.011 | 0.011 | 0.041 | [0.009; 0.032] |
| Hareket penceresi | s | 6 | 5.768 | 3.951 | 3.775 | 3.176 | 12.800 | [1.806; 9.731] |


## Deney: mode-switch

- Koşu sayısı: **1**
- Hedefe ulaşma oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Mod geçiş başarı oranı | - | 1 | 1.000 | 1.000 | N/A | 1.000 | 1.000 | N/A |
| Mod geçiş gecikmesi (ort) | s | 1 | 0.015 | 0.015 | N/A | 0.015 | 0.015 | N/A |
| Mod geçiş gecikmesi (medyan) | s | 1 | 0.012 | 0.012 | N/A | 0.012 | 0.012 | N/A |
| Mod geçiş gecikmesi (maks) | s | 1 | 0.042 | 0.042 | N/A | 0.042 | 0.042 | N/A |


## Deney: obstacle

- Koşu sayısı: **1**
- Hedefe ulaşma oranı: **%0.0**
- Durum dağılımı: İptal (abort): 1

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 1 | 20.368 | 20.368 | N/A | 20.368 | 20.368 | N/A |
| Planlanan yol uzunluğu | m | 1 | 4.845 | 4.845 | N/A | 4.845 | 4.845 | N/A |
| Gerçek gidilen mesafe | m | 1 | 2.539 | 2.539 | N/A | 2.539 | 2.539 | N/A |
| Ortalama hız | m/s | 1 | 0.125 | 0.125 | N/A | 0.125 | 0.125 | N/A |
| Doğrusal konum hatası | m | 1 | 2.380 | 2.380 | N/A | 2.380 | 2.380 | N/A |
| Açısal konum hatası | ° | 1 | 120.243 | 120.243 | N/A | 120.243 | 120.243 | N/A |
| Yeniden planlama sayısı | - | 1 | 10.000 | 10.000 | N/A | 10.000 | 10.000 | N/A |
| Alınan plan mesajı | - | 1 | 15.000 | 15.000 | N/A | 15.000 | 15.000 | N/A |
| İlk yeniden planlama süresi | s | 1 | 6.395 | 6.395 | N/A | 6.395 | 6.395 | N/A |
| Engel algılama olayı | - | 1 | 21.000 | 21.000 | N/A | 21.000 | 21.000 | N/A |
| Nav2 recovery sayısı | - | 1 | 4.000 | 4.000 | N/A | 4.000 | 4.000 | N/A |
| BT plan hesaplama | - | 1 | 28.000 | 28.000 | N/A | 28.000 | 28.000 | N/A |
| BT costmap temizleme | - | 1 | 0.000 | 0.000 | N/A | 0.000 | 0.000 | N/A |
| BT geri çekilme | - | 1 | 1.000 | 1.000 | N/A | 1.000 | 1.000 | N/A |
| Harita kapsama oranı | oran | 1 | 0.375 | 0.375 | N/A | 0.375 | 0.375 | N/A |
| Bilinen harita alanı | m² | 1 | 534.950 | 534.950 | N/A | 534.950 | 534.950 | N/A |
| Dolu hücre sayısı | - | 1 | 22257.000 | 22257.000 | N/A | 22257.000 | 22257.000 | N/A |


## Deney: repeatability

- Koşu sayısı: **10**
- Hedefe ulaşma oranı: **%70.0**
- Durum dağılımı: İptal (abort): 3, Başarılı: 7

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 10 | 62.424 | 61.102 | 35.566 | 19.600 | 130.503 | [36.983; 87.864] |
| Planlanan yol uzunluğu | m | 10 | 9.578 | 7.895 | 4.976 | 4.575 | 18.799 | [6.019; 13.137] |
| Gerçek gidilen mesafe | m | 10 | 10.805 | 9.253 | 5.906 | 4.598 | 23.701 | [6.581; 15.030] |
| Ortalama hız | m/s | 10 | 0.191 | 0.201 | 0.066 | 0.066 | 0.274 | [0.144; 0.238] |
| Doğrusal konum hatası | m | 10 | 1.336 | 0.180 | 3.673 | 0.136 | 11.788 | [-1.291; 3.963] |
| Açısal konum hatası | ° | 10 | 19.454 | 15.220 | 23.327 | 1.031 | 80.917 | [2.768; 36.140] |
| Yeniden planlama sayısı | - | 10 | 39.600 | 41.000 | 18.787 | 14.000 | 69.000 | [26.162; 53.038] |
| Alınan plan mesajı | - | 10 | 47.600 | 46.500 | 26.672 | 15.000 | 98.000 | [28.522; 66.678] |
| İlk yeniden planlama süresi | s | 10 | 1.420 | 1.308 | 0.418 | 1.199 | 2.604 | [1.121; 1.719] |
| Engel algılama olayı | - | 10 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| Nav2 recovery sayısı | - | 10 | 2.600 | 0.500 | 3.596 | 0.000 | 10.000 | [0.028; 5.172] |
| BT plan hesaplama | - | 10 | 93.300 | 91.500 | 53.329 | 28.000 | 194.000 | [55.153; 131.447] |
| BT costmap temizleme | - | 10 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 10 | 0.400 | 0.000 | 0.699 | 0.000 | 2.000 | [-0.100; 0.900] |
| Harita kapsama oranı | oran | 10 | 0.367 | 0.366 | 0.005 | 0.360 | 0.374 | [0.363; 0.370] |
| Bilinen harita alanı | m² | 10 | 525.355 | 524.321 | 4.573 | 521.220 | 533.353 | [522.083; 528.626] |
| Dolu hücre sayısı | - | 10 | 20875.200 | 20970.500 | 767.102 | 19960.000 | 21989.000 | [20326.486; 21423.914] |


## Mecanum yön testi — yön bazlı ölçümler

Her yön ayrı koşu olarak ölçüldü; hareketi operatör joystick ile yaptı, araç yalnızca başlangıç/bitiş pozundan hesapladı. Beklenen bileşen **kalın** okunmalıdır: ileri/geri komutunda ileri sütunu, yanal komutta yanal sütunu, dönüşte dönme sütunu. Diğer sütunlar istenmeyen **sapma**dır.

| Yön | İleri (cm) | Yanal (cm) | Dönme (°) | Mesafe (cm) | Pencere (s) | Sonuç |
|---|---|---|---|---|---|---|
| İleri (+x) | +60.1 | -2.4 | -5.5 | 60.2 | 3.18 | DOGRU |
| Geri (−x) | -67.0 | +1.1 | -1.9 | 67.0 | 3.35 | DOGRU |
| Sola yanal (+y) | +1.5 | +71.1 | -5.6 | 71.1 | 3.74 | DOGRU |
| Sağa yanal (−y) | -4.1 | -83.5 | -4.2 | 83.6 | 4.16 | DOGRU |
| Saat yönü tersi (+wz) | -2.0 | -0.5 | +41.2 | 2.0 | 7.38 | DOGRU |
| Saat yönü (−wz) | -0.6 | -1.1 | -13.8 | 1.3 | 12.80 | DOGRU |

**Sonuç: 6/6 yön doğru.** Hareket pencereleri fiziksel olarak tutarlıdır (ölçülen mesafe / pencere, 0.75 m/s tavanının altında).


## Hedef bazlı tekrarlanabilirlik


### camera — ortam: lab-gercek — hedef: kamera-60s

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Kamera FPS | FPS | 1 | 14.996 | 14.996 | N/A | 14.996 | 14.996 | N/A |
| Kareler arası süre | s | 1 | 0.067 | 0.067 | N/A | 0.067 | 0.067 | N/A |
| Toplam kesinti | s | 1 | 0.000 | 0.000 | N/A | 0.000 | 0.000 | N/A |
| Alınan kare sayısı | - | 1 | 900.000 | 900.000 | N/A | 900.000 | 900.000 | N/A |


### mapping — ortam: lab-gercek — hedef: harita-baslangic

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Harita kapsama oranı | oran | 1 | 0.372 | 0.372 | N/A | 0.372 | 0.372 | N/A |
| Bilinen harita alanı | m² | 1 | 527.450 | 527.450 | N/A | 527.450 | 527.450 | N/A |
| Dolu hücre sayısı | - | 1 | 20170.000 | 20170.000 | N/A | 20170.000 | 20170.000 | N/A |
| Haritalama süresi | s | 1 | 40.784 | 40.784 | N/A | 40.784 | 40.784 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-1-ileri

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.602 | 0.602 | N/A | 0.602 | 0.602 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.024 | 0.024 | N/A | 0.024 | 0.024 | N/A |
| Hareket penceresi | s | 1 | 3.176 | 3.176 | N/A | 3.176 | 3.176 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-2-geri

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.670 | 0.670 | N/A | 0.670 | 0.670 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.011 | 0.011 | N/A | 0.011 | 0.011 | N/A |
| Hareket penceresi | s | 1 | 3.354 | 3.354 | N/A | 3.354 | 3.354 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-3-sol

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.711 | 0.711 | N/A | 0.711 | 0.711 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.015 | 0.015 | N/A | 0.015 | 0.015 | N/A |
| Hareket penceresi | s | 1 | 3.744 | 3.744 | N/A | 3.744 | 3.744 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-4-sag

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.836 | 0.836 | N/A | 0.836 | 0.836 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.041 | 0.041 | N/A | 0.041 | 0.041 | N/A |
| Hareket penceresi | s | 1 | 4.159 | 4.159 | N/A | 4.159 | 4.159 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-5-ccw

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.020 | 0.020 | N/A | 0.020 | 0.020 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.020 | 0.020 | N/A | 0.020 | 0.020 | N/A |
| Hareket penceresi | s | 1 | 7.377 | 7.377 | N/A | 7.377 | 7.377 | N/A |


### mecanum — ortam: lab-gercek — hedef: yon-6-cw

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal mesafe | m | 1 | 0.013 | 0.013 | N/A | 0.013 | 0.013 | N/A |
| Yanal/ikincil sapma | m/° | 1 | 0.013 | 0.013 | N/A | 0.013 | 0.013 | N/A |
| Hareket penceresi | s | 1 | 12.800 | 12.800 | N/A | 12.800 | 12.800 | N/A |


### mode-switch — ortam: lab-gercek — hedef: mod-10tekrar

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Mod geçiş başarı oranı | - | 1 | 1.000 | 1.000 | N/A | 1.000 | 1.000 | N/A |
| Mod geçiş gecikmesi (ort) | s | 1 | 0.015 | 0.015 | N/A | 0.015 | 0.015 | N/A |
| Mod geçiş gecikmesi (medyan) | s | 1 | 0.012 | 0.012 | N/A | 0.012 | 0.012 | N/A |
| Mod geçiş gecikmesi (maks) | s | 1 | 0.042 | 0.042 | N/A | 0.042 | 0.042 | N/A |


### obstacle — ortam: lab-gercek — hedef: engel-1

- Tekrar sayısı: **1**
- Başarı oranı: **%0.0**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 1 | 20.368 | 20.368 | N/A | 20.368 | 20.368 | N/A |
| Planlanan yol uzunluğu | m | 1 | 4.845 | 4.845 | N/A | 4.845 | 4.845 | N/A |
| Gerçek gidilen mesafe | m | 1 | 2.539 | 2.539 | N/A | 2.539 | 2.539 | N/A |
| Ortalama hız | m/s | 1 | 0.125 | 0.125 | N/A | 0.125 | 0.125 | N/A |
| Doğrusal konum hatası | m | 1 | 2.380 | 2.380 | N/A | 2.380 | 2.380 | N/A |
| Açısal konum hatası | ° | 1 | 120.243 | 120.243 | N/A | 120.243 | 120.243 | N/A |
| Yeniden planlama sayısı | - | 1 | 10.000 | 10.000 | N/A | 10.000 | 10.000 | N/A |
| Alınan plan mesajı | - | 1 | 15.000 | 15.000 | N/A | 15.000 | 15.000 | N/A |
| İlk yeniden planlama süresi | s | 1 | 6.395 | 6.395 | N/A | 6.395 | 6.395 | N/A |
| Engel algılama olayı | - | 1 | 21.000 | 21.000 | N/A | 21.000 | 21.000 | N/A |
| Nav2 recovery sayısı | - | 1 | 4.000 | 4.000 | N/A | 4.000 | 4.000 | N/A |
| BT plan hesaplama | - | 1 | 28.000 | 28.000 | N/A | 28.000 | 28.000 | N/A |
| BT costmap temizleme | - | 1 | 0.000 | 0.000 | N/A | 0.000 | 0.000 | N/A |
| BT geri çekilme | - | 1 | 1.000 | 1.000 | N/A | 1.000 | 1.000 | N/A |
| Harita kapsama oranı | oran | 1 | 0.375 | 0.375 | N/A | 0.375 | 0.375 | N/A |
| Bilinen harita alanı | m² | 1 | 534.950 | 534.950 | N/A | 534.950 | 534.950 | N/A |
| Dolu hücre sayısı | - | 1 | 22257.000 | 22257.000 | N/A | 22257.000 | 22257.000 | N/A |


### repeatability — ortam: lab-gercek — hedef: rota-A-B

- Tekrar sayısı: **10**
- Başarı oranı: **%70.0**

**Konumlandırma doğruluğu — yalnızca hedefe ULAŞILAN koşular.** İptal edilen bir hedefte hata doğruluk değil, robotun nereye kadar gidebildiğidir; o koşular başarı oranında ve alttaki tüm-koşu tablosunda aynen sayılır.

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Doğrusal konum hatası | m | 7 | 0.173 | 0.178 | 0.032 | 0.136 | 0.222 | [0.144; 0.203] |
| Açısal konum hatası | ° | 7 | 14.636 | 18.142 | 9.407 | 1.270 | 25.591 | [5.935; 23.337] |

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 10 | 62.424 | 61.102 | 35.566 | 19.600 | 130.503 | [36.983; 87.864] |
| Planlanan yol uzunluğu | m | 10 | 9.578 | 7.895 | 4.976 | 4.575 | 18.799 | [6.019; 13.137] |
| Gerçek gidilen mesafe | m | 10 | 10.805 | 9.253 | 5.906 | 4.598 | 23.701 | [6.581; 15.030] |
| Ortalama hız | m/s | 10 | 0.191 | 0.201 | 0.066 | 0.066 | 0.274 | [0.144; 0.238] |
| Doğrusal konum hatası | m | 10 | 1.336 | 0.180 | 3.673 | 0.136 | 11.788 | [-1.291; 3.963] |
| Açısal konum hatası | ° | 10 | 19.454 | 15.220 | 23.327 | 1.031 | 80.917 | [2.768; 36.140] |
| Yeniden planlama sayısı | - | 10 | 39.600 | 41.000 | 18.787 | 14.000 | 69.000 | [26.162; 53.038] |
| Alınan plan mesajı | - | 10 | 47.600 | 46.500 | 26.672 | 15.000 | 98.000 | [28.522; 66.678] |
| İlk yeniden planlama süresi | s | 10 | 1.420 | 1.308 | 0.418 | 1.199 | 2.604 | [1.121; 1.719] |
| Engel algılama olayı | - | 10 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| Nav2 recovery sayısı | - | 10 | 2.600 | 0.500 | 3.596 | 0.000 | 10.000 | [0.028; 5.172] |
| BT plan hesaplama | - | 10 | 93.300 | 91.500 | 53.329 | 28.000 | 194.000 | [55.153; 131.447] |
| BT costmap temizleme | - | 10 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 10 | 0.400 | 0.000 | 0.699 | 0.000 | 2.000 | [-0.100; 0.900] |
| Harita kapsama oranı | oran | 10 | 0.367 | 0.366 | 0.005 | 0.360 | 0.374 | [0.363; 0.370] |
| Bilinen harita alanı | m² | 10 | 525.355 | 524.321 | 4.573 | 521.220 | 533.353 | [522.083; 528.626] |
| Dolu hücre sayısı | - | 10 | 20875.200 | 20970.500 | 767.102 | 19960.000 | 21989.000 | [20326.486; 21423.914] |

