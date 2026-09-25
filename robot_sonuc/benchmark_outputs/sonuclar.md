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

_Bu deney için sayısal metrik ölçülemedi._


## Deney: obstacle

- Koşu sayısı: **2**
- Hedefe ulaşma oranı: **%0.0**
- Durum dağılımı: İptal (abort): 2

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 2 | 13.286 | 13.286 | 10.016 | 6.203 | 20.368 | [-76.706; 103.277] |
| Planlanan yol uzunluğu | m | 2 | 5.297 | 5.297 | 0.639 | 4.845 | 5.749 | [-0.444; 11.037] |
| Gerçek gidilen mesafe | m | 2 | 1.269 | 1.269 | 1.795 | 0.000 | 2.539 | [-14.858; 17.396] |
| Ortalama hız | m/s | 2 | 0.062 | 0.062 | 0.088 | 0.000 | 0.125 | [-0.729; 0.854] |
| Doğrusal konum hatası | m | 2 | 3.530 | 3.530 | 1.627 | 2.380 | 4.681 | [-11.084; 18.145] |
| Açısal konum hatası | ° | 2 | 119.810 | 119.810 | 0.613 | 119.377 | 120.243 | [114.304; 125.315] |
| Yeniden planlama sayısı | - | 2 | 5.000 | 5.000 | 7.071 | 0.000 | 10.000 | [-58.530; 68.530] |
| Alınan plan mesajı | - | 2 | 9.500 | 9.500 | 7.778 | 4.000 | 15.000 | [-60.383; 79.383] |
| İlk yeniden planlama süresi | s | 1 | 6.395 | 6.395 | N/A | 6.395 | 6.395 | N/A |
| Engel algılama olayı | - | 2 | 10.500 | 10.500 | 14.849 | 0.000 | 21.000 | [-122.913; 143.913] |
| Nav2 recovery sayısı | - | 2 | 2.000 | 2.000 | 2.828 | 0.000 | 4.000 | [-23.412; 27.412] |
| BT plan hesaplama | - | 2 | 17.000 | 17.000 | 15.556 | 6.000 | 28.000 | [-122.766; 156.766] |
| BT costmap temizleme | - | 2 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 2 | 0.500 | 0.500 | 0.707 | 0.000 | 1.000 | [-5.853; 6.853] |
| Harita kapsama oranı | oran | 2 | 0.375 | 0.375 | 0.000 | 0.375 | 0.375 | [0.375; 0.375] |
| Bilinen harita alanı | m² | 2 | 534.948 | 534.948 | 0.004 | 534.945 | 534.950 | [534.916; 534.979] |
| Dolu hücre sayısı | - | 2 | 22257.000 | 22257.000 | 0.000 | 22257.000 | 22257.000 | [22257.000; 22257.000] |


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



### obstacle — ortam: lab-gercek — hedef: engel-1

- Tekrar sayısı: **2**
- Başarı oranı: **%0.0**

**Tüm koşular (başarısızlar dahil).**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 2 | 13.286 | 13.286 | 10.016 | 6.203 | 20.368 | [-76.706; 103.277] |
| Planlanan yol uzunluğu | m | 2 | 5.297 | 5.297 | 0.639 | 4.845 | 5.749 | [-0.444; 11.037] |
| Gerçek gidilen mesafe | m | 2 | 1.269 | 1.269 | 1.795 | 0.000 | 2.539 | [-14.858; 17.396] |
| Ortalama hız | m/s | 2 | 0.062 | 0.062 | 0.088 | 0.000 | 0.125 | [-0.729; 0.854] |
| Doğrusal konum hatası | m | 2 | 3.530 | 3.530 | 1.627 | 2.380 | 4.681 | [-11.084; 18.145] |
| Açısal konum hatası | ° | 2 | 119.810 | 119.810 | 0.613 | 119.377 | 120.243 | [114.304; 125.315] |
| Yeniden planlama sayısı | - | 2 | 5.000 | 5.000 | 7.071 | 0.000 | 10.000 | [-58.530; 68.530] |
| Alınan plan mesajı | - | 2 | 9.500 | 9.500 | 7.778 | 4.000 | 15.000 | [-60.383; 79.383] |
| İlk yeniden planlama süresi | s | 1 | 6.395 | 6.395 | N/A | 6.395 | 6.395 | N/A |
| Engel algılama olayı | - | 2 | 10.500 | 10.500 | 14.849 | 0.000 | 21.000 | [-122.913; 143.913] |
| Nav2 recovery sayısı | - | 2 | 2.000 | 2.000 | 2.828 | 0.000 | 4.000 | [-23.412; 27.412] |
| BT plan hesaplama | - | 2 | 17.000 | 17.000 | 15.556 | 6.000 | 28.000 | [-122.766; 156.766] |
| BT costmap temizleme | - | 2 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 2 | 0.500 | 0.500 | 0.707 | 0.000 | 1.000 | [-5.853; 6.853] |
| Harita kapsama oranı | oran | 2 | 0.375 | 0.375 | 0.000 | 0.375 | 0.375 | [0.375; 0.375] |
| Bilinen harita alanı | m² | 2 | 534.948 | 534.948 | 0.004 | 534.945 | 534.950 | [534.916; 534.979] |
| Dolu hücre sayısı | - | 2 | 22257.000 | 22257.000 | 0.000 | 22257.000 | 22257.000 | [22257.000; 22257.000] |


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

