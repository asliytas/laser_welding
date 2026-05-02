# Faz 3 — Kaynak Yolu Tespiti (Başlangıç)

> Bu doküman Faz 3'ün **tamamlanmış hali değildir**. Faz 3, Faz 1 ve 2 gibi temiz
> bir "amaç → adımlar → çıktı" zincirine oturmadı: ortada hâlâ çözemediğimiz
> kenar durumları var. Bu README, hangi yaklaşımları denediğimizi, neyi neden
> seçtiğimizi, neyi neden bıraktığımızı ve şu an nerede takılı kaldığımızı —
> 5.A'dan 5.K'ya kadarki iterasyonların kayıtlı bir günlüğü olarak — anlatır.
> Devamı (manuel mod iyileştirmeleri ve gap senaryolarının çözümü) ayrı bir
> belgede yazılacak.

## Amaç

Faz 2 sonunda elimizde, kullanıcının iki body'yi mouse ile seçtiği ve bounding
box validation'dan geçirdiği bir araç vardı. Faz 3'ün amacı bunun üstüne
**"iki parça arasındaki kaynak yolunun otomatik olarak çıkarılması"** katmanını
eklemekti. Somut hedef: kullanıcı seçimini onayladıktan sonra, sistem
ekranda kaynak hattının geometrik karşılığını (tek bir eğri ya da bir
eğri zinciri) **renkli bir polyline** olarak göstermeli; bu eğri Faz 4'te
trajectory planning için sayısal olarak dışarı verilebilmeli.

Üst seviyede pipeline şu hale gelecekti:

```
Phase 2 confirmed bodies
       ↓
   Find weld geometry  (Phase 3)
       ↓
   Display polyline + export to JSON
       ↓
   Trajectory planning  (Phase 4)
```

Faz 3'ün asıl zorluğu, "iki parça arasındaki kaynak yolu" tanımının
geometriye göre çok farklı şeyler olabilmesidir:

- İki düz plakanın yan yana getirildiği bir butt joint → **tek bir doğru parçası**.
- L köşesi (T-joint, lap joint) → **iki düzlemin kesişim çizgisi**.
- Silindir içine yerleştirilmiş bir mil (cylinder-in-hole) → **kapalı bir çember**
  ya da iç içe iki çember.
- Bir koninin yamuk silindire oturması → **eğrisel bir çember/elips**.
- Birbirine değmeyen ama yakın iki parça (clearance fit) → kaynaklanacak edge'ler
  ayrı body'lerde, dolgu kaynağı (plug/fillet weld) gerekiyor.
- Sadece tek bir noktada temas eden iki yüzey (point contact) → tek nokta.

Faz 3, bu çeşitliliği tek bir algoritma ile değil, **bir öncelik sırasına
sokulmuş birden çok yöntemle** ele almayı denedi. Aşağıdaki günlük, bu
yöntemlerin hangi sırayla denendiğini ve hangilerinden vazgeçildiğini anlatır.

## Mimari Karar — Body Pair Selection'dan FACE Pair Selection'a Geçiş (5.A)

Faz 2'nin en büyük kararı "face → body" yönündeydi. Faz 3'ün ilk büyük
kararı bunun **tersi** oldu: body seçiminden face seçimine geri dönmek.

### Faz 2'nin getirdiği yapı

Faz 2 sonunda kullanıcı **iki body** seçiyordu. Plan, Faz 3'te bu iki body
arasındaki kaynak yolunu doğrudan `BRepAlgoAPI_Section(body_a, body_b)`
ile hesaplamaktı. Section, OpenCASCADE'in iki shape'in kesişimindeki edge'leri
döndüren standart yöntemidir.

### Karşılaşılan Sorun

Touching/coincident geometrilerde Section sonucu **anlamlı bir wire vermiyordu**.
İki yüzey birbirine tam yapıştığında (Body Pair'de bu çok yaygın), Section
sayısız küçük edge ile dolu, kendiyle çakışan ve `ShapeAnalysis_FreeBounds`'in
"bağlanamayan kenar" hataları verdiği bir compound döndürüyordu. Tipik debug
çıktısı şuna benziyordu:

```
[ALGO] Section returned 142 raw edges
[ALGO] ShapeAnalysis_FreeBounds: 0 wires (cannot connect)
```

Sebep: body-body kesişim, **paylaşılan tüm yüzeyleri** içeriyordu.
Birbirinin üzerine düşmüş iki plakada bu, sadece kaynak hattı değil, plakaların
**tüm temas yüzeyi** demekti. Kullanıcının "şu kenardan kaynak yapılacak"
niyeti bu sonuçtan ayıklanamıyordu.

### Karar

Body Pair konsepti tamamen bırakıldı. Yeni mantık: **kullanıcı her body'den
bir face seçer**, sistem `BRepAlgoAPI_Section(face_a, face_b)` ile iki yüzeyin
buluşma yerinin edge'lerini hesaplar. Bu kararın iki önemli sonucu oldu:

- **Semantik olarak doğru.** Bir kaynak fiziksel olarak iki yüzeyin buluşma
  yeridir, iki katı cismin değil. "Hangi yüze kaynak istiyorsun?" sorusu
  daha net bir niyet ifadesi.
- **Faz 2'de "yapılamadı" denilen face selection geri geldi.** Faz 2'de selection
  mode bug'ı vardı (`SetSelectionModeFace()` çalışmıyordu). 5.A'da
  `context.Activate(ais, SEL_MODE_FACE)` yaklaşımı denendi ve bu sefer çalıştı —
  her AIS_Shape için ayrı ayrı face mode aktive edildiği zaman picking gerçekten
  face döndürmeye başladı. Bu, Faz 2'deki `_display.SetSelectionModeFace()`
  yaklaşımının neden başarısız olduğunu da geriye dönük açıkladı: o çağrı
  global modu değiştiriyordu ama AIS_Shape bazlı default mode hâlâ SOLID idi.

5.A'dan itibaren Faz 2'nin Body Pair akışı (4.B kodu) **deprecated** sayıldı,
arşivde kaldı. Manual edge picking ikinci bir mod olarak korundu — Auto/Manual
radio butonları ile geçiş yapılıyor.

## Adımlar — 5.A'dan 5.K'ya İterasyon Günlüğü

Faz 3 boyunca dosya isimleri `main_updated_5.A.py`, `main_updated_5.B.py`, …
şeklinde sürdü. Her sürüm bir önceki sürümün tam kopyası olarak başladı, sadece
o iterasyondaki sorun adreslendi. Sürümler arası boş geçen tek harf 5.J oldu
(üzerinde çalışılırken kararsızlığa düşüldü, içeriği 5.K'ya taşındı).

### 5.A — Face Pair Selection altyapısı

Yukarıdaki mimari karar uygulandı. Kullanıcı bir Body'den bir face tıklar,
diğer Body'den bir face tıklar; sistem ikisini sarı/mavi renklerle highlight
eder, bir "Confirm Face Pair" butonu kaynak yolunu hesaplar. Bu sürümün asıl
algoritması hâlâ `BRepAlgoAPI_Section(face_a, face_b)` idi.

**Geçti.** Düzgün ve yapışık geometrilerde (yan yana iki plaka, bir L-joint)
section çağrısı kısa, anlamlı bir edge listesi döndürdü.

**Takıldı.** Aynı geometrinin bazı varyasyonlarında section yine bağlı bir wire
veremedi: STEP dosyasında her iki body'nin **aynı edge'i paylaştığı**
durumlarda (shape sharing), section bu edge'i 0 uzunlukta hesaplıyordu.
Yani iki yüz aynı kenardan paylaşılıyorsa "kesişim sıfır" oluyordu.

### 5.B — Shared Edge Tabanlı Algoritma Sırası

5.A'daki bug aslında bir ipucu verdi: STEP shape sharing dolayısıyla bazı
geometrilerde iki face zaten **aynı TopoDS_Edge** referansını paylaşıyordu.
Bunu Section'a sokmak yerine, doğrudan ortak edge'i bulmak çok daha sağlam
olurdu.

5.B'de algoritma sırası şu hale geldi:

1. **Topological shared edge.** `face_a` ve `face_b`'nin edge'leri tek tek
   gezilir, `TopoDS_Edge.IsSame()` ile karşılaştırılır. Aynı referans olan
   edge'ler doğrudan kaynak yolu olarak alınır. Şekil paylaşımı çalışan
   STEP dosyalarında çok hızlı.
2. **Geometric coincident edge.** Shape sharing yoksa, edge çiftleri
   uzunluk + endpoint + `BRepExtrema_DistShapeShape` mesafesine göre eşlenir
   (toleranslar `COINCIDENT_LEN_TOL=0.01`, `COINCIDENT_ENDPOINT_TOL=0.1`,
   `COINCIDENT_DIST_TOL=0.1`).
3. **Section fallback.** İlk iki yöntem boş dönerse `BRepAlgoAPI_Section`
   son çare olarak çağrılır. Bu kuyruğun sonunda olmasının sebebi: Section
   approximated arc'lar üretebiliyor ve bunların görsel doğrulaması zor.
4. **Hata mesajı.** Üçü de boşsa "Manual mode öner" dialog'u çıkar.

Ek olarak, tek-edge senaryoları için `BRepBuilderAPI_MakeWire` fast-path
eklendi; `ConnectEdgesToWires` çağrılmadan tek bir wire kuruluyor (eski
"connect 0 edges" hatasını by-pass ediyor).

5.B'de ayrıca **multi-segment "Add Segment"** akışı eklendi. Tek bir
"Confirm Face Pair" yerine, kullanıcı her face çiftinden sonra "Add Segment"
butonuna basıyor; L-joint, kutu kenarları gibi geometrilerde aynı path'e
birden çok segment ekleniyor. "Finish Path" toplam path'i finalize ediyor,
"Clear All Segments" sıfırlıyor.

**Geçti.** Düz plaka çiftleri, T-joint, basit L-joint geometrileri için yol
artık güvenle çıkıyordu.

### 5.C — Segment Manager (Faz 4 köprüsü)

5.B'de eklenen segment kavramı, kullanıcının ekleme düzenini sonradan
düzeltebileceği bir UI gerektirdi. 5.C bu UI'yi getirdi:

- `QListWidget` tabanlı segment listesi: tıkla → o segment'i highlight et,
  delete, move up/down.
- **Undo Last Segment** (Ctrl+Z).
- Klavye kısayolları: Enter (Add), Esc (Reset Faces), Del (delete),
  Ctrl+O/S/L/E (file ops).
- Her segment dict'inde `start_point`, `end_point`, `is_closed` saklandı —
  Faz 4'te trajectory için.
- **Continuity check.** Ardışık iki segment arası `CONTINUITY_GAP_TOL=0.5 mm`
  kontrolü; gap varsa list item'ında uyarı.
- **Duplicate detection.** Aynı endpoint'lere sahip yeni bir segment "duplicate"
  uyarısı verir (forward + reverse karşılaştırma).
- Section fallback ile gelen segment'ler list'te italic+bold + "⚠ verify
  visually" prefix.
- JSON Save / Load / Export-for-trajectory.

**Karar.** Proximity check (bbox tabanlı) artık **engelleyici değil**.
Faz 2'deki "bbox çakışmıyorsa Confirm yapamazsın" politikası kaldırıldı;
sadece sarı uyarı çıkar, kullanıcı dilerse devam eder. Sebep: bridging
senaryolarına hazırlık (parçalar arasında gap varsa bbox çakışmayabilir
ama yine kaynak hattı kurulması mümkün).

### 5.D — Clearance Fit + İlk Bridging Denemesi

Buraya kadar tüm algoritma "iki yüz birbirine değiyor" varsayımı üzerine
kuruluydu. Real-world kaynak senaryolarının önemli bir kısmında parçalar
arasında küçük bir boşluk (clearance fit) bulunur — fillet ya da plug welding
yapılır. 5.D ilk bridging denemesini ekledi:

- `CLEARANCE_THRESHOLD_MM = 5.0`. Bu altında bir gap varsa "plug/fillet
  welding ipucu" verilir.
- `_try_closest_edge_bridging`: face_a × face_b edge'lerinde en yakın çift,
  gap ∈ [0.05, 5.0] mm + uzunluk farkı < 0.5 mm.
- Operatöre `QMessageBox.question` ile "Bridging segment ekleyeyim mi?"
  diye sorar; Yes derse Body A edge'i kullanılır (genelde levha tarafı).
- Method etiketi: "Bridging (gap X.XX mm)" — list'te italic+bold visual warning.

**Geçti.** Tek bir bridging adayı olan basit clearance fit'lerde çalıştı.

**Takıldı.** İki dairesel parça arasında (şaft içinde delik) bridging tek bir
adaya zorlanıyordu, ama gerçekte iki ayrı dairesel kaynak yolu vardı.
Algoritma "en yakın 1 çift" politikası ile yetersiz kaldı.

### 5.E — Multi-bridging (parallel candidates)

5.D'nin kısıtlaması açıktı: bir face çiftinde birden çok bridging adayı
olabiliyordu. 5.E'de:

- `_try_closest_edge_bridging` → `_find_bridging_candidates` (LIST of dicts döner).
- `BRIDGING_LEN_REL_TOL = 0.15` — mutlak yerine **%15 göreceli** uzunluk farkı
  kullanılır. Sebep: dairesel clearance fit'lerde dış ve iç çevreler birkaç mm
  ayrı çapta olabiliyor, mutlak 0.5 mm tolerans bunları reddediyordu.
- `BridgingCandidatesDialog(QDialog)` — checkbox'lı seçim arayüzü, default
  hepsi seçili.
- `_apply_bridging_batch(candidates, selected_indices)` — kullanıcının seçtiği
  N adayı tek seferde batch olarak ekler.
- Eski tek-aday Yes/No dialog'u kaldırıldı.

**Geçti.** Şaft-delik senaryosu artık iki dairesel segment veriyordu.

**Takıldı.** Hâlâ "her bir aday → bir segment" mantığı. Aslında kullanıcının
beklediği şey **Body A tarafı bir path, Body B tarafı bir path** olarak iki
ayrı eğri ailesiydi. 4 adayın hepsi seçilince 4 ayrı dairesel segment
ekleniyordu — 2 path olarak gruplanmıyordu.

### 5.F — Path-Based Bridging (paradigma değişikliği)

5.E'deki gruplama eksikliği yapısal bir karar gerektirdi. 5.F'de "edge-pair"
konsepti tamamen "Path A / Path B" konseptine çevrildi:

- `_find_bridging_paths`: face_a için karşı face'e yakın edge'leri toplar ve
  `_build_wires` ile birleştirir → **tek bir path**. Aynısı face_b için.
  Sonuçta her iki face için 1 path döner (bazen 1, bazen 2 path).
- `BridgingPathsDialog`: 1–2 satır (Path A / Path B), default `Qt.Unchecked` —
  bilinçli karar zorlamak için.
- `_apply_bridging_paths`: her path = TEK segment (içindeki edge sayısı önemsiz).
- Method etiketi: "Bridging-A (gap X.XX mm)" / "Bridging-B" — hangi taraftan
  geldiği net.
- Renk paleti değişti: `SEGMENT_PALETTE` 8 sabit yüksek-kontrast renk
  (kırmızı, siyah, koyu mavi, mor, koyu yeşil, turuncu, koyu pembe, turkuaz).
  Eski kırmızı→sarı gradient, segment sayısı arttıkça birbirine girip ayırt
  edilemez hale geliyordu.
- Continuity check güncellendi: aynı side'dan ardışık iki bridging segment
  (`Bridging-A` → `Bridging-A`) `_segment_gap` 0.0 dönerek connected sayılır.

**Geçti.** "İki ayrı path" mantığı kullanıcı niyetine daha yakın geldi.

### 5.G — Bridging Duplicate Filter

5.F'de fark edilen bir bug: kullanıcı aynı face çiftini iki kez seçince,
`_apply_bridging_paths` her seferinde aynı path'i tekrar ekliyordu. 4 dialog
× 4 segment = 16 segment, hepsi aynı dairesel kaynak yolu.

- 5.G'de `add_segment` bridging branch'i: dialog açılmadan **önce** path'leri
  `_bridging_path_already_added` ile filtreler.
- Hepsi zaten ekliyse → "Already Added" info dialog (hangi segment olduğunu
  söyler).
- Sadece YENİ path'ler dialog'da listelenir.
- `_apply_bridging_paths` içinde defense-in-depth duplicate check.
- `BridgingPathsDialog`: tek path tespit edildiğinde sarı uyarı kutusu —
  "diğer body'nin weld path'i muhtemelen farklı bir face üzerinde, oraya da
  bakman gerek".

**Geçti.** Kullanıcı tekrar tıklamalarında temiz davranış.

### 5.H — Path B False-Duplicate Bug Fix

5.G'deki duplicate filter, konsantrik (cylinder-in-hole) geometrilerde
**KRİTİK** bir bug çıkardı:

- Path A → R=12 mm dış çember, length 75.4 mm, start_point ekvatorda.
- Path B → R=11.8 mm iç çember, length 74.1 mm, start_point yine ekvatorda.
- İki path'in start_point'leri arasındaki radyal mesafe **0.2 mm**.
- `DUPLICATE_TOL = 0.5 mm` → Path B yanlışlıkla Path A'nın duplicate'i
  sayılıyordu.
- Sonuç: konsantrik kaynaklarda kullanıcı sadece 1 segment görüyordu, 2 değil.

Düzeltme: `_bridging_path_already_added` **side-aware** yapıldı. Sadece aynı
method prefix'li (Bridging-A vs Bridging-A) mevcut segmentlerle karşılaştırır;
Path A ekli olsa bile Path B yine eklenebilir. Ek olarak
`BRIDGING_LEN_DUP_TOL = 1.0 mm` uzunluk kontrolü: iki bridging path'in
gerçekten aynı olabilmesi için hem endpoint'ler yakın hem de uzunlukları
benzer olmalı.

UI tarafında da bir terminoloji değişikliği oldu (kullanıcı isteği):
"path" → "segment"/"weld segment". "Body N side weld segment" formatı.
İç method etiketleri (`Bridging-A`/`Bridging-B`) aynı kaldı — kod logic için
gerekiyor. Dialog default state `Qt.Unchecked` → `Qt.Checked` (clearance
fit'te kullanıcı genelde hepsini ekliyor zaten).

### 5.I — Çoklu Weld Modeli ve Gap Edge Pairs

5.F-G-H boyunca biriken yapısal hisle birlikte 5.I büyük bir refactor
getirdi: artık sadece "tek bir path biriktiriyoruz" değil, **çoklu weld**
yönetimi var.

Yeni veri yapısı:

```python
self.welds = [
    {"id": ..., "name": ..., "context": ..., "segments": [...]},
    ...
]
self.active_weld_id = ...
```

Sağ panelde "Paths" başlıklı bir QListWidget eklendi — kullanıcı:

- "+ New Path" ile yeni weld açar (ayrı bir kaynak hattı).
- Listeden bir weld seçince segment listesi o weld'in segmentlerini gösterir.
- Rename, Delete butonları her weld için.
- "Add Segment" aktif weld'e segment ekler.

Bu, gerçek kaynak senaryolarında bir parçada **birden fazla bağımsız kaynak
hattı** olabileceği gerçeğini modelliyor (örneğin bir kapağın 4 kenarı 4 ayrı
hat olarak yönetilebilir, ya da bir flanjda iç ve dış çember iki ayrı weld).

Bridging yaklaşımı da değişti. 5.F-H'deki "_find_bridging_paths" yerine
**`_find_gap_edge_pairs`** geldi:

- Karşılıklı yakın edge çiftleri tek tek toplanır (5.E'deki gibi),
  ama bu sefer her çift **iki ayrı weld'e** dağıtılır:
  Body A tarafı edge'i Path A weld'ine, Body B tarafı edge'i Path B weld'ine.
- `GapEdgePairsDialog` — checkbox'lı liste, "Pair 1 — Body 1: 75.4 mm |
  Body 2: 74.1 mm | gap 0.20 mm [recommended]".
- `_mark_recommended_gap_pairs`: tam karşılıklı seçildiyse ana seam'i
  varsayılan seçili işaretle, kısa yan kenarları işaretsiz bırak. Heuristic:
  uzunluk gruplarına bak, dengeli grup yapısı varsa "full boundary" olarak
  düşün, hepsini seçili yap.

Yeni eklenen iki kontrol:

- **`_face_normal_at_midpoint` + `_faces_suitable_for_gap_paths`.** Gap path
  uygulamadan önce iki face'in normal vektörlerinin birbirini gördüğünü test
  ediyor (`PLANAR_FACING_DOT_MIN = 0.35`). Birbirine bakmıyorlarsa (örn. iki
  paralel yan yüz, ortada parça yok) bridging skip edilir, "faces are not
  facing" dialog'u çıkar.
- **`_find_point_contact`.** İki yüz birbirine sadece bir noktada değiyorsa
  (örneğin küre-düzlem teması, koni ucunun plakaya değmesi), shared/coincident
  edge yok ama gap < 1e-3 mm. Bu durumda `BRepExtrema_DistShapeShape`
  + face vertex'lerini cluster'lar; tek bir benzersiz nokta varsa kullanıcıya
  "tek nokta kaynak segmenti ekleyeyim mi?" diye sorar.

5.I'nın özeti git commit'te şu cümlelerle yakalanmıştı:

> **5.I gerçekten iyi çalışıyor. ancak koni gibi bir geometriyi algılayamıyor.**

### 5.K — Manuel Arayüz İyileştirmeleri ve Bridging Path Sadeleştirmesi

5.K iki yönde değişiklik getirdi.

**Bir:** 5.F'den beri yan kolda duran "path-based bridging" altyapısı
(`BridgingPathsDialog`, `_find_bridging_paths`, `_face_boundary_path`,
`_find_gap_boundary_paths`, `_apply_bridging_paths`, `_bridging_path_already_added`,
`_apply_gap_paths_as_body_paths`) tamamen kaldırıldı. 5.I'da getirilen
`_find_gap_edge_pairs` + `GapEdgePairsDialog` zaten daha basit ve doğru
sonucu veriyordu; iki paralel altyapı bakım yükü oldu, eski kod silindi.
Kod tabanı 2884 satırdan 2519 satıra indi.

**İki:** Manual mode iyileştirildi. 5.A'dan beri Manual mode "hemen segmenti
ekle" şeklinde çalışıyordu — kullanıcı bir edge'e tıklıyordu, anında segment
listesine giriyordu, geri almak için Ctrl+Z gerekiyordu. Bu kullanıcıya
geriye dönük hissi veriyordu. 5.K'da:

- `_segment_from_wire_info`, `_manual_segments_from_edges`,
  `_display_manual_preview_segment`, `_remove_manual_preview_segment`,
  `_reassign_manual_preview_colors` yardımcıları eklendi.
- Önizleme akışı: kullanıcı edge tıklar → preview olarak görünür
  (`manual_pending_segments`'a gider, ekrana çizilir ama segment listesine
  girmez) → "Apply Manual Path" ile commit eder.
- "Clear Edges" butonu preview'ları temizler.
- Manual mode aktifken sadece edge picking ile ilgili butonlar görünür;
  "Reset Faces", "Add Segment" gibi auto-mode butonları gizlenir.
- Tek bir edge seçildiğinde de `_build_wires` çağrılır — single edge'den de
  düzgün bir wire/segment çıkarılabiliyor (eski "wire build failed" durumu
  ortadan kalktı).

5.K commit mesajı, projenin şu anki durumunu açıkça ifade ediyor:

> **main_updated_5.K.py dosyasında iç içe geçmiş (joint, overlap vs)
> geometriler dışında bütün touching geometrilerde çalışıyor. Bazı
> geometriler (mesela koni gibi) kod içerisinde tanınamıyor. birbirinden
> uzakta duran geometrilerde ise düzgün bir yaklaşım kurulamadı. kodda bu
> noktadan sonra manuel modda iyileştirmeler yapılacak.**

## Şu An Takılı Kaldığımız Noktalar

5.K'da kararlı haldeyiz ama Faz 3 hâlâ "tamamlanmış" sayılamaz. Açık
kalan üç sorun var:

### 1. İç içe geçmiş geometriler (joint / overlap)

İki body'nin sadece yapışık değil, biri diğerinin **içine girmiş** olduğu
durumlar (örneğin bir flanjın bir kovanın içine geçirilmesi, bir milin
deliğe sıkı geçmesi):

- Topological shared edge: bazen var, bazen yok (STEP'in nasıl yazıldığına
  bağlı).
- Geometric coincident edge: ortak edge'lerin geometrisi, bir face'in
  diğerinin **içinde** kaldığı için endpoint'ler eşleşmiyor.
- Section fallback: çakışan yüzeylerden dolayı 5.A'daki bug geri geliyor —
  yine bağlı wire kuramıyor.

Şu an bu durumlar genelde "Manual mode öner" hatasıyla sonuçlanıyor.

### 2. Koni ve benzeri eğrisel-konik geometriler

Konik bir parçanın bir düzleme oturması:

- Temas çevresi bir **çember** (koninin tabanına yakınsa) ya da bir
  **elips** (eğik kesilmişse).
- Topological shared edge: STEP'te bu çember koninin alt kenarı olarak
  yazılmışsa bulunabiliyor; ama çoğu CAD aracı koniyi "kapalı bir face"
  olarak değil "açık bir BSplineSurface" olarak yazıyor → ortak edge yok.
- `_find_gap_edge_pairs`: koni edge'lerinin birçoğu (tepe noktasından
  inen meridyen edge'leri) düzlem yüzeyinin edge'leri ile eşleşmiyor —
  uzunluk farkı toleransı çok yüksek.
- `_face_normal_at_midpoint`: koni yüzeyinin normal'i u,v parametrelerine
  göre değişiyor; midpoint'te alınan normal "facing" testini geçmeyebiliyor.

Sonuç: koni-içeren geometrilerde sistem genelde "no shared edge" diyor.
Manual mode'da kullanıcı koninin alt çemberini doğrudan seçtiğinde çözülüyor,
ama otomatik tespit yapılamıyor.

### 3. Birbirinden uzakta duran geometriler

5.D-H'deki bridging ve 5.I'daki gap edge pairs, `BRIDGING_MAX_DIST_MM = 5.0`
ile sınırlı. Gerçek bir tipik welding senaryosunda boşluk genelde 0.1–2 mm,
ama tasarım kontrolü için 5–20 mm aralığında parçalarla da çalışmak isteyen
kullanıcılar var. Şu anki algoritmalarımız:

- Gap > 5 mm olduğunda hiçbir aday üretmiyor.
- "Yüzler birbirine uzak" dialog'u çıkıyor, kullanıcıdan Manual moda geçmesi
  isteniyor.
- Ama Manual mod tek bir edge'i seçtiriyor — uzaktaki iki edge arasında
  "filler" çizecek bir araç yok.

Bu konuda **yapısal bir karar daha veremedik**. Bridging mesafesini büyütmek
yanlış pozitifleri (kasıtsız edge eşleşmeleri) artırıyor; küçük tutmak
geometrileri kaçırıyor. Adaptive bir threshold (face boyutuna göre) ya da
operatöre boşluk eşiğini sordurmak değerlendiriliyor — şu an karara
bağlanmadı.

## Vazgeçilen Yollar

- **Body Pair Selection (Faz 2'den miras).** 5.A'da bırakıldı, gerekçesi yukarıda.
- **Section'ı birincil yöntem yapmak (5.A).** 5.B'de fallback'e demote edildi,
  topological shared edge öne geçti.
- **Tek-aday bridging Yes/No dialog'u (5.D).** 5.E'de checkbox dialog'a
  taşındı; tek aday durumlarında bile checkbox arayüzü tutarlılık sağladı.
- **Edge-pair konsepti olarak bridging (5.E).** 5.F'de Path A/B yapısına
  yükseltildi. Sonra 5.I'da yine edge-pair benzeri yapıya döndü ama bu sefer
  iki ayrı weld'e dağıtarak.
- **Path-based bridging dialog (5.F-H).** 5.K'da tamamen silindi; 5.I'nın
  GapEdgePairsDialog'u tek doğru çözüm olarak kaldı.
- **`DUPLICATE_TOL` tek başına yeterli (≤ 5.G).** 5.H'de side-aware
  duplicate check eklendi; salt mesafeye dayanan duplicate kontrolü konsantrik
  geometrilerde başarısız.
- **Engelleyici proximity check (5.B'ye kadar).** 5.C'de uyarı seviyesine
  düşürüldü; bridging senaryolarına yer açtı.
- **Renk gradyanı (5.B-E).** 5.F'de 8 sabit yüksek-kontrast palet ile
  değiştirildi; segment sayısı 4'ü geçince ayırt edilemez oluyordu.
- **Manual mode'un anında commit etmesi (≤ 5.I).** 5.K'da preview +
  "Apply Manual Path" akışı geldi — kullanıcı yanlış edge seçince Ctrl+Z'ye
  basmak zorunda kalmıyor.

## Faz 3 Şu Anki Çıktıları (5.K itibariyle)

- **Auto mode** (face pair → 4 algoritmalı tespit):
  - Topological shared edge → çoğu touching geometri için anında çözüm.
  - Geometric coincident edge → STEP'te shape sharing yoksa fallback.
  - Section → karmaşık intersection'lar için son çare, görsel uyarıyla.
  - Point contact → tek nokta temas için özel branch.
  - Gap edge pairs (clearance fit, 0.05–5 mm) → çift weld olarak (Path A/B).
- **Manual mode** (edge pick → preview → Apply):
  - Tek edge'den de wire kurulur.
  - Birden fazla edge önizlemesi biriktirilebilir, "Apply" ile commit edilir.
- **Çoklu weld yönetimi:** sağ panelde "Paths" listesi; her weld bağımsız bir
  kaynak hattı, kendi segment listesi var.
- **Segment manager:** delete, move up/down, undo, click→highlight, Ctrl+Z,
  duplicate detection, continuity check, otomatik segment sıralaması (en iyi
  süreklilik için).
- **JSON Save/Load/Export-for-trajectory:** Faz 4'e veri taşıma yolu hazır;
  her segment için `start_point`, `end_point`, `is_closed`, `length`, `type`,
  `method` alanları.
- **Operator-friendly hata mesajları:** "no shared edge", "faces not facing",
  "faces too far", "point contact?" — her birinde Manual mode önerisi.
- **Yan panel:** mode toggle, status, face X/Y etiketleri, proximity bilgisi,
  segment listesi, weld listesi, kontrol butonları.

Test edilen pipeline:

```
STEP file → Bodies displayed → Pick face A + face B
   ↓
   Topological  →  Geometric  →  Section  →  Point contact?  →  Gap pairs?
   ↓
   1+ weld'e dağıtılan segment(ler) → live preview → list
   ↓
   JSON / Trajectory export
```

## Faz 3'ün Devamı

Bu README, **5.K itibariyle** Faz 3'ün başlangıç kısmının kaydıdır. Bundan
sonraki adımlar şu üç başlıkta toplanıyor:

1. **Manual mode'un derinleştirilmesi.** Kullanıcı edge'leri zincir halinde
   seçebilmeli, gap'leri köprüleyici doğru parçaları manuel olarak ekleyebilmeli,
   ek olarak iki edge arasındaki kısa mesafeyi otomatik kapatabilmeli. 5.K'da
   gelen preview altyapısı bu işin başlangıcı.
2. **Koni ve eğrisel konik yüzeylerin tanınması.** Sürface tipini sorgulayan
   (`GeomAbs_Cone`, `GeomAbs_Cylinder`, `GeomAbs_Sphere`, vb.) özel handler'lar.
   `_face_surface_type` zaten kodda var ama henüz dispatch edilmiyor.
3. **Adaptive gap threshold.** Bridging mesafesini face boyutuna oranlı bir
   parametre yapmak ya da kullanıcıya geometriye göre sordurmak.

Faz 4 (trajectory planning) bu üç sorun çözülmeden başlatılmayacak; Faz 3'ün
çıktısı yeterince kararlı olmazsa robot tarafına aktarılan path da güvenilmez
olur. Şu anki durum tipik geometrilerin büyük bir kısmında çalışıyor —
özellikle düz plakalar, T-joint, lap joint, basit silindirik geçmeler ve
clearance fit'ler — ama yukarıdaki üç edge-case Faz 3'ün "complete" olarak
işaretlenebilmesinden önce ele alınması gereken adımlar.
