# Robot Performans Testi Sonuçları

- Toplam koşu sayısı: **18**
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

_Bu deney için sayısal metrik ölçülemedi._


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



### mecanum — ortam: lab-gercek — hedef: yon-2-geri

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**



### mecanum — ortam: lab-gercek — hedef: yon-3-sol

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**



### mecanum — ortam: lab-gercek — hedef: yon-4-sag

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**



### mecanum — ortam: lab-gercek — hedef: yon-5-ccw

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**



### mecanum — ortam: lab-gercek — hedef: yon-6-cw

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**



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

