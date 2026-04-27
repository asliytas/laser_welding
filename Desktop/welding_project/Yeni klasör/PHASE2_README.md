# Faz 2 — İnteraktif Seçim ve Doğrulama

## Amaç

Faz 1, kullanıcının modeli sadece görüntüleyip döndürebildiği "izleyici" bir uygulama bırakmıştı. Faz 2'nin amacı, bu izleyici uygulamayı **etkileşimli bir araca** dönüştürmekti: kullanıcı modelin hangi iki parçası arasında kaynak yapılacağını mouse ile seçebilmeli, sistem seçimi görsel olarak doğrulamalı, yan panelde anlık geri bildirim vermeli ve kaynak yapılamayacak senaryoları (örneğin birbirine değmeyen iki parça) önceden uyarmalıydı.

Faz sonunda elimizde olması beklenen şey: STEP dosyasını açıp iki parçayı tıklayarak seçen, seçimi sarıya boyayan, validation'dan geçirip "Confirm" butonu ile sonraki faza hazır hale getiren bir araç.

## Mimari Karar — Face Selection'dan Body Selection'a Geçiş

Bu fazın en önemli kararı, mimari hedefin orta yerinde değişmesidir. Bu karar fazın geri kalan tüm adımlarını şekillendirdiği için ayrı bir başlık altında, açıkça anlatılması gerekir.

### Orijinal Plan: Face Selection

Faz 2'nin başında plan **face (yüzey) seçimi** üzerine kuruluydu. Kullanıcı bir parçanın belirli bir yüzeyine tıklayacak, sistem o yüzeyi tanıyacak; iki yüzey seçildikten sonra Faz 3'te bu iki yüzey arasındaki kesişim eğrisi hesaplanacaktı. "Hangi yüzü kaynaklamak istiyorsun?" sorusunu kullanıcıya doğrudan sormak en hassas yaklaşım gibi görünüyordu.

### Karşılaşılan Sorun

Picking mekanizmasını kuracak adımı (2.2) yazdığımızda, OpenCASCADE'in `SetSelectionModeFace()` çağrısının umulan etkiyi göstermediğini fark ettik. Kullanıcı bir yüze tıkladığında picking sistemi face yerine **solid (body)** geri döndürüyordu. Bunu doğrulamak için `mousePressEvent` içine debug satırları eklendi:

```python
print(f"DEBUG: picked shape type = {selected_shape.ShapeType()}")
print(f"DEBUG: picked shape hash = {hash(selected_shape)}")
```

Çıktı durumu net gösterdi: `ShapeType = 2` (yani `TopAbs_SOLID`), face beklendiği gibi `4` (`TopAbs_FACE`) değildi. Üstelik hash değerleri sürekli iki farklı değer arasında dönüyordu — yani iki body için iki ayrı nesne, hangi yüze tıklarsan tıkla aynı body referansları geliyordu. Picking face moduna girmiyor, solid modunda kalıyordu.

### Çözüm Denemeleri

Birkaç farklı yol denendi:

- `_display.SetSelectionModeFace()` doğrudan çağrı — etki etmedi.
- `context.Deactivate()` + `context.Activate(4)` ile manuel mod değişimi — etki etmedi.
- Her `AIS_Shape` için ayrı `Activate(ais_shape, 4)` çağrısı — etki etmedi.
- pythonocc-demos repo'sundaki `core_geometry_face_recognition_from_stepfile.py` örneğinden `register_select_callback` mekanizması — fetch işlemi sırasında alınamadı, sonradan denenmek üzere bekletildi.

Aynı zamanda farklı bir teknik sorun daha ortaya çıktı: `DisplayShape` çağrıldığında OpenCASCADE shape'i kopyalıyor olduğu için, dosyadan okuduğumuz shape ile sahnedeki shape **fiziksel olarak farklı nesneler** oluyordu. Bu yüzden `IsSame()` karşılaştırması bile başarısız oluyor, picking'den dönen shape ile bizim listemizdeki face'leri eşleştirmek zorlaşıyordu. `MapShapesAndAncestors` ile bir face→solid haritası kurulabilirdi, ama bu da temel sorunu (face moduna geçilememesi) çözmüyordu.

### Üç Yol Üzerinde Karar

Bu noktada üç seçenek değerlendirildi:

- **A — Face selection'ı çözmeye gitmek:** pythonocc forumlarına dalmak, AIS_Shape üzerinden manuel selection mode aktivasyonu denemek. Avantajı: orijinal plana uygun, daha hassas seçim. Dezavantajı: bilinmeyen süre, takılma riski; bu pythonocc sürümünde gerçekten desteklenmiyor olma ihtimali vardı.
- **B — Body selection'a geçmek:** Picking solid döndürüyorsa, bunu reddetmek yerine kullanmak. "Hangi yüzü?" yerine "Hangi iki parça?" sormak. Avantajı: çalışan kod var, momentum kaybı yok. Dezavantajı: karmaşık geometrilerde "hangi kenardan kaynak yapılacak" belirsizliği kalır.
- **C — Şimdilik olduğu gibi bırakıp tezin sonunda "geliştirilmesi gereken" diye yazmak:** Akademik açıdan dürüst ama tez savunmasında "neden yapmadın?" sorusunu davet eden bir yol.

İlk düşünüş B'ye yöneldi. Sonra bir ara A'ya kayıldı çünkü tezin başlığında "precise trajectories" geçiyordu ve hassasiyet gerekçesiyle face seçimi tercih edilebilirdi. Ardından nihai karar **önce B, sonra A** şeklinde belirlendi: önce sunumda gösterilebilecek çalışan bir prototip ortaya çıkacak (B), sonra zamanı varsa face selection da eklenebilecekti (A). Yazılım mühendisliğinin "önce baseline, sonra refine" prensibi bu kararı destekliyordu.

### Kararın Gerekçeleri

Body selection'ın pratikte yeterli olduğuna dair argümanlar şunlardı:

- Tezin gerçek problemi face seçmek değil, **kaynak yolunu çıkarmak**. Face seçimi yalnızca bir araçtı; body seçiminden de aynı yola otomatik face matching ile ulaşmak mümkündü.
- Tipik bir kaynak senaryosunda iki parça (örneğin üst ve alt plaka) bellidir; "bu iki parçayı kaynakla" demek kullanıcı için zaten net bir komut olur. Manuel face seçimi gereksiz bir detay seviyesi olabilir.
- "Kullanıcıya minimum yük, sisteme maksimum otonom" yaklaşımı modern engineering software felsefesine uygun bir tasarım kararıdır ("intent-based selection").
- Geri dönüş her zaman mümkündü — face selection ileride ekleneceği zaman mimari aynı kalacak, sadece input katmanı değişecekti.

### Etkisi

Karar verildikten sonra Faz 2'nin geri kalan adımları (2.3–2.6) tamamen body üzerine yazıldı: highlight body bazlıdır, FIFO kuyruğu body indeksleri tutar, validation iki body'nin bounding box'larını karşılaştırır, panel "Body 1 / Body 2" listeler. Faz 3 de bu kararı miras aldı; iki body arasındaki kesişim doğrudan `BRepAlgoAPI_Section` ile hesaplandı.

Face selection'a sonradan dönmek için bir not düşüldü; mimaride bunu mümkün kılacak ayrım korundu (validation ve seçim katmanları, hesaplama katmanından bağımsız tutuldu).

## Adımlar

### Adım 2.1 — Mouse Tıklamasını Yakalama

Henüz picking yok, sadece "Qt event sistemi mouse tıklamalarını yakalıyor mu?" testi. `qtViewer3d`'den miras alan bir `WeldingViewer` sınıfı yazıldı; `mousePressEvent` metodu override edildi ve içine `print` satırı kondu. Override sırasında `super().mousePressEvent(event)` çağrısı korundu; aksi halde parent sınıfın orijinal davranışı (kameranın drag ile döndürülmesi) bozulurdu.

Doğrulama testi: Viewer'a tıklandığında terminale "click at (x, y)" mesajı çıkması. Geçti.

### Adım 2.2 — Picking (Tıklanan Şeyi Tanımlama)

Bu adım fazın **teknik olarak en kritik kısmıydı**. Plan: kullanıcı tıkladığında `_display.MoveTo(x, y)` ile pozisyon bildirilir, ardından `_display.Select()` ile o noktada seçim yapılır, sonra `context.SelectedShape()` ile seçilen şey alınır.

Selection mode başlangıçta `SetSelectionModeFace()` ile face'e ayarlanmaya çalışıldı. Yukarıda "Mimari Karar" bölümünde anlatılan sorun burada ortaya çıktı: picking face yerine solid döndürüyordu. Karar verilip body selection'a geçildikten sonra mantık şu hale geldi:

- Kullanıcı tıklar.
- Picking bir `TopoDS_Solid` döndürür.
- `solids` listesinde dolaşılarak `solid.IsSame(picked_shape)` ile karşılaştırılır.
- Eşleşen body'nin indeksi (1, 2, 3, ...) bulunur.

Doğrulama testi: Farklı body'lere tıkladığında terminalde doğru body indeksinin yazılması; boş alana tıkladığında "No shape under cursor" çıkması. Üç farklı body'li test dosyasında doğrulandı.

### Adım 2.3 — Highlight (Görsel Vurgulama)

Picking çalışınca, kullanıcının doğru parçayı seçtiğinden emin olabilmesi için görsel geri bildirim eklendi. Mantık:

- `DisplayShape` aslında bir AIS_Shape listesi döndürür; bu döndürülen listeyi sakladık (`self.ais_shapes`).
- Her body'nin orijinal rengi de saklandı (`self.body_colors`).
- Tıklama gelince `context.SetColor(ais_shape, highlight_color, True)` ile rengi sarıya çevrildi; tekrar tıklayınca orijinal renge geri döndürüldü (toggle).

Sınıf seviyesinde bir `HIGHLIGHT_COLOR` sabiti tanımlandı. `selected_bodies` set'i ile o anki seçim durumu takip edildi; bir body bu set'teyse "seçili" demekti.

Doğrulama testi: Bir body'ye tıklandığında sarıya dönmesi; tekrar tıklandığında eski rengine dönmesi. Geçti.

### Adım 2.4 — State Management (FIFO Politikası)

Welding senaryosu **tam olarak iki body** seçimi gerektiriyordu — bir az değil, bir fazla değil. 3. body'ye tıklandığında ne olacağı sorusuna üç farklı politika önerildi:

- **Politika A (Katı):** "İki body seçili, önce birini bırak" uyarısı çıksın. Açık ama rahatsız edici.
- **Politika B (FIFO):** En eski seçim atılsın, yeni body kuyruğa girsin. Kullanıcı sürekli düzeltiyor olabilir, akıcı.
- **Politika C (LIFO):** En yeni seçim değiştirilsin, eski kalsın. "Son tıklamamı düzeltiyorum" hissi.

**Politika B (FIFO)** seçildi. Welding'de "ah, asıl şu parçayı seçmeliydim" durumu sık olduğu için akıcılık öne çıktı; ayrıca atılan seçimin görsel olarak eski rengine dönmesi kullanıcıya hemen bilgi veriyordu, sürpriz yaratmıyordu.

`selected_bodies` bir liste olarak tutuldu (set değil — sıralama önemliydi); kuyruk dolu ise `pop(0)` ile en eski atılıyor, yeni body `append` ile ekleniyordu. Toggle mantığı korundu: zaten seçili bir body'ye tekrar tıklamak onu kaldırıyordu.

Doğrulama testi: Üç farklı body'ye sırayla tıklayıp terminalde `(queue full — dropped oldest: Body 1)` mesajının çıktığını görmek; aynı body'ye iki kez tıklayıp toggle'ın çalıştığını teyit etmek. Çoklu body içeren büyük bir test dosyasında doğrulandı.

### Adım 2.5 — Yan Panel (UI)

Şimdiye kadar tüm geri bildirim ya viewer'daki renk değişimi ya da terminal çıktısıydı. Profesyonel görünüm için yan panel eklendi. Pencerenin solunda 3D viewer, sağında bir panel:

- "Selected (n/2)" sayacı — kullanıcı kuyruğun durumunu görür.
- Seçili body'lerin listesi.
- "Reset Selection" butonu — tüm seçimi temizler, body'ler eski rengine döner.
- "Confirm Selection" butonu — sadece tam 2 body seçiliyken aktif olur.

Pencere genişliği panele yer açmak için 1200×700'e çıkarıldı. `_update_panel`, `_select_body`, `_deselect_body` gibi yardımcı metotlar (Python konvansiyonu gereği `_` ile başlayan "private" metotlar) sınıfın sonuna yerleştirildi. Bu organizasyon — kurulum metotları → ana iş metotları → yardımcılar — kod büyüdükçe okunabilirliği korudu.

Doğrulama testi: Bir body seçildiğinde panelde listelenmesi, Confirm butonunun ancak 2 body seçildiğinde aktifleşmesi, Reset'in tüm durumu sıfırlaması. Geçti.

### Adım 2.6 — Validation (Bounding Box Kontrolü)

Son adım, kullanıcı Confirm'e basmadan önce seçimin **mantıklı** olduğundan emin olmaktı. Birbirinden uzaktaki iki parça için kaynak hattı oluşturulamaz; sistem bunu önceden yakalayıp kullanıcıyı uyarmalıydı.

Bunun için bounding box (Bnd_Box) kontrolü kullanıldı: her body için OCC, eksen-hizalı en küçük kapsayan kutuyu hesaplar. İki kutunun çakışıp çakışmadığını test etmek için `Bnd_Box.IsOut(other_box)` metodu var — `False` dönerse kutular çakışıyor, yani parçalar büyük olasılıkla temas halinde.

Bu yaklaşımın **kaba ama hızlı** bir filtre olduğu kabul edildi:

- Bbox çakışmıyorsa → kesinlikle temas yok (validation kesin başarısız).
- Bbox çakışıyorsa → büyük olasılıkla temas var (Faz 3'te asıl kesişim hesabıyla netleşecek).

Validation `confirm_selection` içine yerleştirildi: çakışma yoksa `QMessageBox.warning` ile sarı bir uyarı popup'ı çıkıyor; çakışma varsa mavi bir info popup'ı çıkıp Faz 3'e geçişi onaylıyordu. Türkçe karakter politikası gereği tüm popup metinleri İngilizceydi.

Doğrulama testi: Beş body içeren bir test dosyasında üç senaryo denendi:

1. Birbirinden uzaktaki Body 5 ↔ Body 3 → "spatially separated, no welding contact" uyarısı.
2. Yine uzak olan Body 4 ↔ Body 3 → aynı uyarı (sistem tutarlı).
3. Üst üste duran Body 2 ↔ Body 1 → "are confirmed for welding" onayı, terminalde "bounding boxes overlap — likely in contact" mesajı.

Üç senaryo da beklendiği gibi çalıştı. Bu, validation'ın gerçek senaryo çeşitliliğinde test edildiği anlamına geliyordu.

## Yan Konular ve Vazgeçilen Yollar

**`MapShapesAndAncestors` ile face→solid haritası.** Face selection denemeleri sırasında `DisplayShape`'in shape'i kopyalama sorununa karşı bir çözüm olarak `topexp.MapShapesAndAncestors` ile bir global face→solid haritası kurulması düşünüldü. Bu, ana shape üzerinden çalıştığı için kopyalama sorununu by-pass edebilirdi. Ancak body selection'a geçildikten sonra bu yapıya gerek kalmadı — picking zaten doğrudan solid döndürüyordu, bir adımda eşleşme mümkündü. Kod terkedildi.

**`Set` vs `List` veri yapısı seçimi.** İlk denemede `selected_bodies` bir `set` olarak tutulmuştu (üyelik kontrolü O(1) olduğu için). Ancak FIFO politikası seçimin **sırasını** bilmek istediği için (`pop(0)` ile en eskiyi atmak) liste yapısına geçildi. Üyelik kontrolü O(n) oldu ama liste hep en fazla 2 elemanlı olduğu için fark önemsizdi.

**Aynı body'ye iki kez seçim engeli.** Toggle mantığı zaten aynı body'nin iki kez seçilmesini imkansız kılıyordu — ilk tıklamada seçildi, ikinci tıklamada toggle ile kalktı. Bu yüzden "iki face de aynı body'den" senaryosuna karşı ayrı bir validation gerekmedi (face selection planında bu kontrol açıkça yer alıyordu, body selection'da otomatik halloldu).

**Türkçe karakterler.** Faz 1'de alınan karar Faz 2'de de korundu: `QMessageBox` metinleri, panel etiketleri ve terminal mesajları tamamen İngilizce yazıldı. Hem encoding sorunlarına karşı sigorta, hem de tezin uluslararası standartlarda sunulabilmesi için.

**Büyük dosya açılışında ilerleme göstergesi.** 180 body içeren büyük bir test dosyası ile çalışırken, `DisplayShape` döngüsünün uzun sürdüğü ve uygulamanın "donmuş" gibi göründüğü fark edildi. Bir progress bar veya yükleme animasyonu eklemek "yapılması iyi olur" listesine eklendi ama Faz 2'nin kapsamından çıkarıldı; performans optimizasyonu daha sonraki bir polish adımı için bırakıldı.

## Faz 2'nin Çıktıları

Faz 2 sonunda eldeki şeyler:

- STEP dosyasını okuyup görüntüleyen, kullanıcının iki body'yi mouse ile tıklayarak seçebildiği bir uygulama.
- FIFO mantığı ile çalışan, tam 2 body seçimi tutan bir state yönetimi.
- Seçili body'leri sarıya boyayan, deselect'te eski rengine döndüren highlight sistemi.
- Yan panel — anlık seçim listesi, Reset ve Confirm butonları.
- Bounding-box tabanlı, kaynak yapılamayacak senaryoları yakalayan validation katmanı.
- Geçerli bir seçim onaylandığında Faz 3'e geçişi tetikleyen mavi onay popup'ı; geçersiz seçim için sarı uyarı popup'ı.

Test edilen pipeline:

```
Click → Pick (solid) → Body indexing → Highlight + State (FIFO) →
Panel update → Confirm → Validation (bbox) → Phase 3 trigger
```

Bu, "kullanıma hazır bir tool" seviyesinde — tezde ekran görüntüleri ile sunulabilir, jüriye canlı demo verilebilir bir aşamaya geldi. Faz 3, body kesişimi üzerine kuruldu; ileride face selection'a geçiş yapılmak istenirse, validation ve seçim katmanları hesaplama katmanından bağımsız tutulduğu için bu mümkün olacak şekilde tasarlandı.
