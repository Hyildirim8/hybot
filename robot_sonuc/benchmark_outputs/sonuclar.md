# Robot Performans Testi Sonuçları

- Toplam koşu sayısı: **10**
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


## Deney: repeatability

- Koşu sayısı: **8**
- Hedefe ulaşma oranı: **%75.0**
- Durum dağılımı: İptal (abort): 2, Başarılı: 6

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 8 | 70.341 | 65.706 | 35.611 | 19.600 | 130.503 | [40.565; 100.117] |
| Planlanan yol uzunluğu | m | 8 | 9.477 | 7.895 | 4.981 | 4.575 | 18.799 | [5.312; 13.642] |
| Gerçek gidilen mesafe | m | 8 | 12.218 | 10.237 | 5.776 | 5.376 | 23.701 | [7.389; 17.047] |
| Ortalama hız | m/s | 8 | 0.197 | 0.222 | 0.073 | 0.066 | 0.274 | [0.136; 0.258] |
| Doğrusal konum hatası | m | 8 | 0.171 | 0.164 | 0.032 | 0.136 | 0.222 | [0.145; 0.198] |
| Açısal konum hatası | ° | 8 | 11.004 | 11.219 | 8.542 | 1.031 | 22.014 | [3.862; 18.146] |
| Yeniden planlama sayısı | - | 8 | 44.000 | 43.000 | 18.516 | 14.000 | 69.000 | [28.517; 59.483] |
| Alınan plan mesajı | - | 8 | 53.500 | 51.000 | 26.753 | 15.000 | 98.000 | [31.131; 75.869] |
| İlk yeniden planlama süresi | s | 8 | 1.462 | 1.313 | 0.463 | 1.199 | 2.604 | [1.075; 1.850] |
| Engel algılama olayı | - | 8 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| Nav2 recovery sayısı | - | 8 | 3.125 | 1.500 | 3.871 | 0.000 | 10.000 | [-0.111; 6.361] |
| BT plan hesaplama | - | 8 | 105.125 | 100.000 | 53.456 | 28.000 | 194.000 | [60.427; 149.823] |
| BT costmap temizleme | - | 8 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 8 | 0.500 | 0.000 | 0.756 | 0.000 | 2.000 | [-0.132; 1.132] |
| Harita kapsama oranı | oran | 8 | 0.365 | 0.364 | 0.004 | 0.360 | 0.370 | [0.361; 0.368] |
| Bilinen harita alanı | m² | 8 | 523.387 | 522.541 | 2.181 | 521.220 | 526.165 | [521.563; 525.210] |
| Dolu hücre sayısı | - | 8 | 20602.625 | 20562.000 | 576.074 | 19960.000 | 21254.000 | [20120.939; 21084.311] |


## Hedef bazlı tekrarlanabilirlik


### camera — ortam: lab-gercek — hedef: kamera-60s

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Kamera FPS | FPS | 1 | 14.996 | 14.996 | N/A | 14.996 | 14.996 | N/A |
| Kareler arası süre | s | 1 | 0.067 | 0.067 | N/A | 0.067 | 0.067 | N/A |
| Toplam kesinti | s | 1 | 0.000 | 0.000 | N/A | 0.000 | 0.000 | N/A |
| Alınan kare sayısı | - | 1 | 900.000 | 900.000 | N/A | 900.000 | 900.000 | N/A |


### mapping — ortam: lab-gercek — hedef: harita-baslangic

- Tekrar sayısı: **1**
- Başarı oranı: **N/A**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Harita kapsama oranı | oran | 1 | 0.372 | 0.372 | N/A | 0.372 | 0.372 | N/A |
| Bilinen harita alanı | m² | 1 | 527.450 | 527.450 | N/A | 527.450 | 527.450 | N/A |
| Dolu hücre sayısı | - | 1 | 20170.000 | 20170.000 | N/A | 20170.000 | 20170.000 | N/A |
| Haritalama süresi | s | 1 | 40.784 | 40.784 | N/A | 40.784 | 40.784 | N/A |


### repeatability — ortam: lab-gercek — hedef: rota-A-B

- Tekrar sayısı: **8**
- Başarı oranı: **%75.0**

| Metrik | Birim | n | Ortalama | Medyan | Std | Min | Maks | %95 GA |
|---|---|---|---|---|---|---|---|---|
| Hedefe ulaşma süresi | s | 8 | 70.341 | 65.706 | 35.611 | 19.600 | 130.503 | [40.565; 100.117] |
| Planlanan yol uzunluğu | m | 8 | 9.477 | 7.895 | 4.981 | 4.575 | 18.799 | [5.312; 13.642] |
| Gerçek gidilen mesafe | m | 8 | 12.218 | 10.237 | 5.776 | 5.376 | 23.701 | [7.389; 17.047] |
| Ortalama hız | m/s | 8 | 0.197 | 0.222 | 0.073 | 0.066 | 0.274 | [0.136; 0.258] |
| Doğrusal konum hatası | m | 8 | 0.171 | 0.164 | 0.032 | 0.136 | 0.222 | [0.145; 0.198] |
| Açısal konum hatası | ° | 8 | 11.004 | 11.219 | 8.542 | 1.031 | 22.014 | [3.862; 18.146] |
| Yeniden planlama sayısı | - | 8 | 44.000 | 43.000 | 18.516 | 14.000 | 69.000 | [28.517; 59.483] |
| Alınan plan mesajı | - | 8 | 53.500 | 51.000 | 26.753 | 15.000 | 98.000 | [31.131; 75.869] |
| İlk yeniden planlama süresi | s | 8 | 1.462 | 1.313 | 0.463 | 1.199 | 2.604 | [1.075; 1.850] |
| Engel algılama olayı | - | 8 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| Nav2 recovery sayısı | - | 8 | 3.125 | 1.500 | 3.871 | 0.000 | 10.000 | [-0.111; 6.361] |
| BT plan hesaplama | - | 8 | 105.125 | 100.000 | 53.456 | 28.000 | 194.000 | [60.427; 149.823] |
| BT costmap temizleme | - | 8 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | [0.000; 0.000] |
| BT geri çekilme | - | 8 | 0.500 | 0.000 | 0.756 | 0.000 | 2.000 | [-0.132; 1.132] |
| Harita kapsama oranı | oran | 8 | 0.365 | 0.364 | 0.004 | 0.360 | 0.370 | [0.361; 0.368] |
| Bilinen harita alanı | m² | 8 | 523.387 | 522.541 | 2.181 | 521.220 | 526.165 | [521.563; 525.210] |
| Dolu hücre sayısı | - | 8 | 20602.625 | 20562.000 | 576.074 | 19960.000 | 21254.000 | [20120.939; 21084.311] |

