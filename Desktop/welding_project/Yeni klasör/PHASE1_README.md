# Faz 1 — CAD Okuma ve 3D Görselleştirme

## Amaç

Faz 1'in temel hedefi, projenin geri kalanının üzerine inşa edileceği bir **temel altyapı** kurmaktı. Somut olarak şu üç şeyi yapabilen bir masaüstü uygulaması elde etmek istiyorduk:

1. Bir STEP (.step / .stp) dosyasını disk üzerinden okumak.
2. Dosyanın içindeki body'leri (solid'leri) ayırt edip her birinin geometrik özelliklerini (face/edge sayıları) çıkarmak.
3. Bu modeli, kullanıcının döndürüp yakınlaştırabileceği bir 3D pencerede göstermek; her body'yi farklı bir renkte vererek ayırt etmek.

Faz 1 sonunda elimizde "Open STEP" menüsü olan, dosya seçildikten sonra modeli interaktif bir 3D sahnede gösteren bir uygulama olacaktı. Henüz seçim, kaynak yolu hesaplama veya robot komutu üretme yoktu — bu fazın amacı pipeline'ın **giriş ucunu** sağlam kurmaktı.

## Mimari Kararlar

Fazın başında birkaç teknoloji seçimi yapmak zorundaydık. Bu seçimler sonraki tüm fazları doğrudan etkiledi.

**Geometrik kernel olarak OpenCASCADE.** STEP, IGES gibi CAD formatlarını okuyan, BRep tabanlı geometriyi temsil eden ve üzerinde Boolean/section gibi işlemler yapabilen açık kaynak bir kernel. Python'dan erişim için `pythonocc-core` binding'ini kullanmaya karar verdik. Alternatif (FreeCAD'in Python API'si veya doğrudan trimesh gibi mesh tabanlı kütüphaneler) düşünüldü ama mesh tabanlı yaklaşım sonraki fazlarda yapacağımız kesişim hesabı için yeterince hassas olmazdı; FreeCAD ise bağımlılıkları ağır ve ayrı bir uygulamanın içine gömülü olduğu için pratik değildi. OpenCASCADE doğrudan kernel'a erişim sağlıyordu, bu da kontrolü tamamen bizde tutuyordu.

**GUI framework olarak Qt (PyQt5).** İlk planda PyQt6 düşünülmüştü, ancak `pythonocc-core`'un `qtViewer3d` modülü kurulum aşamasında PyQt5 ile sorunsuz çalışırken PyQt6 ile uyum sorunları çıkarma riski vardı. Daha az sürpriz olsun diye PyQt5'te karar kıldık. Bu kararı sonraki fazlarda hiç sorgulamak zorunda kalmadık.

**Paket yönetimi olarak conda.** `pythonocc-core` pip'te bulunmuyor, sadece conda-forge kanalından dağıtılıyor. Bu yüzden Anaconda/Miniconda zorunluydu. Geliştirme ortamı için sistem genelinde değil, izole bir conda environment kullanmaya karar verdik — bu karar daha sonra çok işimize yarayacaktı çünkü `pythonocc-core` belirli OpenCASCADE sürümlerine bağlı ve sistem geneli kurulum yapsaydık başka projelerle çakışırdık.

## Adımlar

Fazı altı küçük adıma böldük. Her adımın bir doğrulama testi vardı, böylece bir adımın sonunda gerçekten çalıştığından emin olmadan sonraki adıma geçmiyorduk.

### Adım 1.1 — Ortam Kurulumu

Anaconda kurulup bir `welding` adında izole environment oluşturuldu. İçine Python 3.11, `pythonocc-core` 7.7.2, PyQt5 ve numpy kuruldu. Doğrulama testi şuydu: Python interaktif modunda `from OCC.Core.STEPControl import STEPControl_Reader` import'u hata vermeden geçmeliydi.

Bu adım kağıt üzerinde basitti ama pratikte en uzun süren adım oldu. Karşılaşılan sorunlar:

- **VS Code terminalinde `conda` komutu tanınmıyordu.** Bunun nedeni, Anaconda kurulumu sırasında "Add Anaconda to PATH" seçeneğinin **bilerek** kapatılmasıydı. Anaconda'nın kendi kurulum sihirbazı bu seçenek için "NOT recommended" diyor çünkü sistemde başka bir Python varsa çakışma yaratıyor. Çözüm: VS Code'un içindeki terminal yerine "Anaconda Prompt"u kullanmak. Bu kalıcı bir alışkanlık haline geldi — sonraki tüm conda işlemleri bu prompt üzerinden yapıldı.
- **İlk environment yanlış sürümle oluşturulmuştu.** İlk denemede environment Python 3.8 ile ve `defaults` kanalı kullanılarak oluşturulmuştu. Bu iki şey birden problemdi: `pythonocc-core` Python 3.8 desteğini bırakmıştı ve `defaults` kanalında düzgün bir paket yoktu. Hata mesajında "pinned specs: python=3.8" satırı bunu net gösteriyordu. Çözüm: environment'ı silip Python 3.11 ile ve `-c conda-forge` kanal parametresi ile yeniden oluşturmak. Bu noktadan sonra ilkesel olarak hep `conda-forge` kanalı kullanıldı.
- **Sistemde Python 3.14 vardı.** Sistem Python'u ile çakışma endişesi vardı ama izole conda environment kullanıldığı için aslında hiç sorun olmadı. Bu deneyim, "izolasyon iyi bir şey" prensibinin neden tercih edildiğini somutlaştırdı.

### Adım 1.2 — STEP Dosyasını Okuma (GUI Olmadan)

GUI'ye geçmeden önce, sadece komut satırından çalışan bir `step_reader.py` script'i yazıldı. Script, OpenCASCADE'in `STEPControl_Reader` sınıfını kullanarak verilen yoldaki STEP dosyasını okur, `OneShape()` ile dosyanın kök shape'ini alır ve içindeki body sayısını ekrana yazar.

Doğrulama testi: Bilinen bir test dosyasında doğru body sayısının raporlanması. Inventor'da elle açıp doğrulanan body sayısı, script'in çıktısı ile karşılaştırıldı.

Bu adım büyük ölçüde sorunsuz geçti. Tek dikkat çekici şey, bir STEP dosyasının kök şeklinin doğrudan bir solid listesi olmadığıydı — `TopoDS_Compound` veya `TopoDS_CompoundSolid` gibi sarmalayıcılar içerebiliyordu. Bu yüzden naif bir döngü yerine `TopologyExplorer` kullanmak gerekti; bu, hangi sarmalayıcının içinde olursa olsun tüm `TopoDS_Solid`'leri toplayan bir yardımcıydı.

### Adım 1.3 — Body'leri Tanıma ve Ayırma

Adım 1.2 her body'yi `TopoDS_Solid` olarak listeden çıkarmıştı, ama bu liste daha "ham"dı. Adım 1.3'te her body'ye bir tamsayı kimliği (Body 1, Body 2, ...) atandı ve her birinin `TopologyExplorer` ile face ve edge sayıları sayıldı. Çıktı şu formdaydı:

```
Body 1: 18 face, 30 edge
Body 2: 14 face, 24 edge
```

Doğrulama testi: Inventor'da aynı modelde elle yapılan sayım ile karşılaştırma. Sayılar tuttu. Bu adım, sonraki fazlarda kullanıcının tıkladığı bir geometrik elemanın "hangi body'ye ait olduğu" sorusunun cevaplanabilmesi için gerekli olan ön çalışmaydı; her body için saklanan kimlik, sonraki fazların temel referans noktası oldu.

### Adım 1.4 — Boş PyQt5 Penceresi

CAD tarafını kısa bir süre bir kenara bırakıp, sadece boş bir PyQt5 penceresi açabilen bir `main.py` yazıldı. İçinde "Hello" yazan veya tamamen boş bir QMainWindow.

Doğrulama testi: `python main.py` çağrısı bir pencere açmalı. Geçti. Bu adımın amacı, GUI framework'ünün kurulumunun doğru olduğunu — `qtViewer3d` ile karıştırmadan önce — bağımsız olarak teyit etmekti. Bir sonraki adımda GUI sorunu çıkarsa, bunun PyQt'den mi yoksa OCC entegrasyonundan mı kaynaklandığını ayırt edebilmek için bu basit kontrol önemliydi.

### Adım 1.5 — Boş 3D Sahne Gömme

Bu adım, fazın en sancılı teknik anıydı. Plan, Adım 1.4'teki boş pencerenin merkezine `OCC.Display.qtDisplay.qtViewer3d` widget'ını yerleştirmekti. İlk denemede `qtViewer3d` import edilirken şu hata geldi:

```
ValueError: incompatible backend_str specified: qt-pyqt5
backend is one of : ('pyqt5', 'pyqt6', 'pyside2', 'pyside6', 'wx', 'tk')
```

Sorun şuydu: `pythonocc-core`'un `qtViewer3d` modülü, hangi Qt binding'ini (PyQt5, PyQt6, PySide2, PySide6) kullanacağını import zamanında bilmek istiyor. Bu yüzden `qtViewer3d` import edilmeden **önce** açıkça backend belirtilmek zorunda. Eski belgelerde bu çağrı `load_backend("qt-pyqt5")` şeklindeydi, ama yeni `pythonocc-core` sürümünde parametre adı değişmişti. Hata mesajının kendisi geçerli seçenekleri listelediği için çözüm hızlı oldu: `load_backend("pyqt5")`.

Backend yüklendikten sonra `qtViewer3d` widget'ı pencerenin `centralWidget`'ı olarak yerleştirildi ve `show()` çağrısından **sonra** `InitDriver()` ile OpenGL context'i başlatıldı. Bu sıralama önemliydi — `InitDriver()` show'dan önce çağrılırsa OpenGL context henüz var olmadığı için sessiz başarısızlık oluyordu.

Doğrulama testi: pencere açılır, koyu bir 3D sahne görünür, mouse ile döndürme/zoom çalışır (henüz içinde model olmasa bile).

### Adım 1.6 — Modeli 3D Sahnede Gösterme

Son adım, Adım 1.2'deki STEP okuma kodu ile Adım 1.5'teki 3D viewer'ı birleştirmekti. `QFileDialog.getOpenFileName` ile dosya seçici eklendi ve "File → Open STEP..." menü öğesi yapıldı. Kullanıcı dosya seçince:

1. STEP okunur, body'ler `TopologyExplorer` ile çıkarılır.
2. Sahne `EraseAll()` ile temizlenir.
3. Önceden tanımlı bir renk paleti (gri, mavi, turuncu, yeşil) modulo body sayısı ile her body'ye bir renk atar.
4. Her body, kendi rengi ile `DisplayShape(solid, color=..., update=False)` çağrısı ile sahneye eklenir.
5. Tüm body'ler eklendikten sonra `FitAll()` ile kamera modele odaklar.

`update=False` parametresi her ekleme sonrası sahnenin yeniden çizilmesini engelliyor — sonunda `FitAll()` zaten yeniden çizimi tetikliyor. Birden fazla body olan bir dosyada bu fark belirgin oldu; her body için update=True yapılsaydı yükleme görsel olarak titreşimli olurdu.

Doğrulama testi: bilinen bir STEP dosyası yüklenir, ekranda iki body birbirinden ayırt edilebilir biçimde farklı renklerde görünür, mouse ile döndürülüp yakınlaştırılabilir. Geçti.

## Yan Konular ve Vazgeçilen Yollar

**Türkçe karakter meselesi.** İlk denemelerde GUI'deki bazı metinlerde Türkçe karakterler (ı, ğ, ş) kullanılmıştı. Qt'nin varsayılan font'u bazı sistemlerde bu karakterleri eksik render ediyor ya da Windows console'unda encoding sorunu çıkarıyordu. Karar: GUI'ye yansıyan tüm metinler İngilizce olacak. Bu hem teknik tutarlılık (encoding sorunlarını ortadan kaldırıyor), hem de tezin uluslararası standartlarda sunulabilmesi açısından isabetli oldu. Kod yorumları Türkçe kalabildi, sadece kullanıcının gördüğü metinler İngilizceleştirildi.

**Console-only iş akışı vs. tek-script entegrasyon.** İlk denemelerde "STEP okuyucu ayrı bir modül, GUI ayrı bir modül" şeklinde net bir ayrım planlanmıştı (`step_reader.py` + `main.py`). Pratikte, `step_reader` mantığı yeterince küçüktü ki `main.py`'nin `open_step_file` metodunun içine doğrudan koymak daha sade duruyordu. Ayrı modül planı, kod büyüyene kadar prematüre soyutlama olurdu — vazgeçildi. Sonraki fazlarda dosya yapısı tekrar gözden geçirildiğinde gerekirse refaktör edilecek.

**Body sayısı sınırlandırması.** İlk renk paleti dört rengi (gri, mavi, turuncu, yeşil) içeriyor; ondan fazla body içeren dosyalar için bu paletteki renkler modulo ile tekrar ediliyor. Bu, ileride aynı renkten iki body'nin yan yana düşmesi durumunda görsel ayrımı zorlaştırabilir. Şimdilik kabul edilebilir bir kısıtlama olarak bırakıldı; ileride gerekirse paletin genişletilmesi ya da otomatik renk üreticisinin eklenmesi düşünülebilir, ama Faz 1'in kapsamı için bu önemsiz kaldı.

**Trihedron (eksen göstergesi) ve "Reset View" butonu.** Faz 1'in sonunda, viewer'a küçük kalite iyileştirmeleri (köşede X-Y-Z ekseni gösteren bir trihedron, kamerayı varsayılana döndüren bir buton, vb.) yapılabilirdi. Bu polish adımları "yapılması iyi olur ama Faz 2'yi geciktirmesin" kategorisine konuldu ve şimdilik ertelendi.

## Faz 1'in Çıktıları

Faz 1 sonunda eldeki şeyler:

- İzole, çalışan bir conda environment (`welding`) — sürüm pinleri sayesinde tekrar üretilebilir.
- STEP dosyasından body'leri okuyup numaralandıran ve face/edge sayılarını raporlayan, doğruluğu Inventor ile çapraz kontrol edilmiş bir okuma katmanı.
- "File → Open STEP..." menüsü olan, dosya seçildikten sonra modeli farklı renklerde gösteren bir interaktif 3D viewer.
- Tüm bu işlevleri tek bir `python main.py` çağrısı ile başlatabilen birleşik bir uygulama.

Test edilen pipeline:

```
STEP file → OCC reader → Topology analysis → Qt 3D viewer → Visualized
```

Bu, sonraki fazların üzerine kurulacağı sağlam bir zemindir. Faz 2'de bu zemin üzerine **kullanıcı seçimi** (mouse ile body'ye tıklayıp seçme, highlight, validation) eklenmiştir; Faz 3'te ise seçilen body'ler arasındaki kesişim eğrisi hesaplanıp kaynak yolu olarak işaretlenmiştir.
