#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <HTTPUpdate.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>

// Bu gateway firmware'inin kendi surumu (Modbus cihazinin sistem_versiyon'uyla
// karistirilmamali -- o sayacin/analizorun kendi ic surumu, bu ise ESP32'nin
// calistirdiği .ino kodunun surumu). OTA guncellemeleri bu degeri artirarak yapilir.
#define FW_VERSION "1.0.1"

#define RE_DE_PIN 4
#define RXD2 16
#define TXD2 17
#define WIFI_RESET_PIN 0 // ESP32 devkit'lerdeki BOOT tusu

// WiFi kesintisiz olarak bu sureden uzun surerse (WiFi.reconnect() ise yaramiyorsa)
// cihaz kendini yeniden baslatir -- bazi durumlarda WiFi yiginini sadece tam reboot kurtarir.
#define WIFI_RECONNECT_RESTART_MS (3UL * 60UL * 1000UL)

// Kurulum (AP) modundayken kayitli ag da denenmeye devam eder. Bu olmadan
// cihaz, modem yeniden baslatildiginda kurulum moduna dusup ORADA KALIYORDU:
// acilistaki 15 saniyelik pencere bir modemin acilma suresine yetmiyor, ve eski
// kod AP moduna girdikten sonra kayitli agi bir daha hic denemiyordu. Sahadaki
// bir cihaz icin bu, WiFi 30 saniye sonra geri gelse bile elinde telefonla
// yerinde mudahale demekti.
#define PROVISION_RETRY_INTERVAL_MS (45UL * 1000UL)
#define PROVISION_RETRY_WINDOW_MS   (15UL * 1000UL)

// TLS sertifikasi bu ALAN ADINA duzenlendi; IP ile baglanmak host dogrulamasini
// kirar. Broker artik 8883'te TLS ile dinliyor -- 1883'te broker parolasi her
// baglantida duz metin gidiyordu ve tum telemetri yolda okunabiliyordu.
const char* mqtt_server = "binaryenerji.com";
const uint16_t mqtt_port = 8883;

// Let's Encrypt zincirinin kok sertifikalari. IKISI de gomulu:
// zincir ... -> Root YE -> ISRG Root X2 -> ISRG Root X1 seklinde geliyor ve X2
// kendinden imzali. Yalnizca X1 gomuluyken dogrulama capraz imzali yola bagli
// kaliyordu; X2 de eklenince zincir bir kademe once demirlenebiliyor. Ilk saha
// denemesinde dokuz el sikismasi istemci tarafindan iptal edildi, bu yuzden
// dogrulama yolu olabildigince kisa ve saglam olsun istiyoruz.
static const char ISRG_ROOTS[] PROGMEM = R"CERT(
""" + x1 + """
""" + x2 + """
)CERT";

// Bu cihazin 32-bit (Signed 32, 2 register) degerlerinde hangi kelimenin once geldigi
// bilinmiyor (Excel haritasinda belirtilmemis). Ilk gercek okumada deger absurt
// buyuk/kucuk cikarsa bunu false yapip tekrar deneyin. ANL13'te de ayni belirsizlik
// vardi, orada da bu flag ile cozulmustu.
#define WORDS_HIGH_FIRST true

ModbusMaster node;
WiFiClientSecure espClient;
// Ust uste basarisiz TLS baglantisi sayaci ve dogrulamanin kapatilip
// kapatilmadigi. Asagidaki loop() icinde kullaniliyor.
int tlsBasarisizSayaci = 0;
bool tlsDogrulamaKapali = false;
// Dogrulamasiz TLS'e dusuldukten sonra ne zaman tekrar denenecegi.
// Onceki surumde dusus KALICIYDI: saat sonradan senkronlansa bile cihaz
// oturum boyunca dogrulamasiz kaliyordu. Sahadaki bir cihazi kalici olarak
// zayif modda birakmak, gecici bir aksakligin kabul edilebilir bedeli degil.
unsigned long tlsTekrarDenemeMs = 0;
#define TLS_DOGRULAMA_TEKRAR_MS (10UL * 60UL * 1000UL)
PubSubClient mqttClient(espClient);

struct PowerData {
  float v1, i1, p1, q1, s1, f1;
  float v2, i2, p2, q2, s2, f2;
  float v3, i3, p3, q3, s3, f3;
  float vN, iN, vL12, vL23, vL31;
  float cos1, pf1, thd1, thvd1;
  float cos2, pf2, thd2, thvd2;
  float cos3, pf3, thd3, thvd3;
};

struct EnergyData {
  uint32_t activeWhTuketim, inductiveVArhTuketim, capacitiveVArhTuketim;
  uint32_t activeWhUretim, inductiveVArhUretim, capacitiveVArhUretim;
};

PowerData data;
EnergyData energy;
uint16_t ctRatio = 0;
unsigned long loopCount = 0;
unsigned long wifiDisconnectedSince = 0;

// Asagidaki struct'lar (PeakBlock/DemandBlock/HarmonicsBlock), referans parametre
// alan fonksiyonlarda (readPeakBlock vb.) kullanildigi icin Arduino'nun otomatik
// prototip ekleme noktasindan (ilk fonksiyon tanimindan hemen once) ONCE
// tanimlanmis olmalari gerekiyor -- yoksa "does not name a type" derleme hatasi
// olusuyor (PowerData/EnergyData bu soruna takilmiyor cunku onlari kullanan
// fonksiyonlar parametre olarak degil global degisken olarak kullaniyor).

// Kaynak: "Minimum Maximum Değerler" sayfasi. Sadece acikca etiketlenmis ilk iki
// blok (Tuketim, Uretim/Export) okunuyor -- sayfadaki tekrar eden 3./4. bloklarin
// (11510+, 11760+) anlami dokumante edilmemis, v1'de atlaniyor.
struct PeakBlock {
  float minVln1, minVln2, minVln3, minVn, maxVln1, maxVln2, maxVln3, maxVn;
  float minVll1, minVll2, minVll3, maxVll1, maxVll2, maxVll3;
  float minI1, minI2, minI3, minIn, maxI1, maxI2, maxI3, maxIn;
  float minP1, minP2, minP3, maxP1, maxP2, maxP3;
  float minQ1, minQ2, minQ3, maxQ1, maxQ2, maxQ3;
  float minS1, minS2, minS3, maxS1, maxS2, maxS3;
  float minThvd1, minThvd2, minThvd3, maxThvd1, maxThvd2, maxThvd3;
  float minThid1, minThid2, minThid3, maxThid1, maxThid2, maxThid3;
  float minFreq, maxFreq, minVUnbal, maxVUnbal, minIUnbal, maxIUnbal;
};
PeakBlock peaksTuketim, peaksUretim;

// Kaynak: "Demandlar" sayfasi, ayni Tuketim/Uretim iki-blok deseni.
struct DemandBlock {
  float maxDV1, maxDV2, maxDV3, minDV1, minDV2, minDV3;
  float maxDI1, maxDI2, maxDI3, minDI1, minDI2, minDI3;
  float maxDP1, maxDP2, maxDP3, minDP1, minDP2, minDP3;
  float maxDQ1, maxDQ2, maxDQ3, minDQ1, minDQ2, minDQ3;
  float maxDS1, maxDS2, maxDS3, minDS1, minDS2, minDS3;
  float maxDThvd1, maxDThvd2, maxDThvd3, minDThvd1, minDThvd2, minDThvd3;
  float maxDThid1, maxDThid2, maxDThid3, minDThid1, minDThid2, minDThid3;
};
DemandBlock demandTuketim, demandUretim;

// Kaynak: "Akım/Gerilim Harmonikleri" sayfalari. Bu sayfalar 62. harmonige kadar
// gidiyor ama 33. harmonikten sonra register genisligi 16-bit'ten 32-bit'e
// degisiyor (kilavuzda acik degil) -- v1'de sadece guvenilir/tek-genislikli
// blogu (3.-31. tek harmonikler, hepsi Signed 16) okuyoruz. Bu, guc kalitesi
// raporlamasinda (IEEE 519 vb.) yaygin kullanilan araligi zaten kapsiyor.
#define HARMONIC_COUNT 15 // 3,5,7,...,31 (tek harmonikler)
struct HarmonicsBlock {
  float thd1, thd2, thd3;
  float h[HARMONIC_COUNT][3]; // [harmonik index][faz 0=L1,1=L2,2=L3]
};
HarmonicsBlock harmCurrent, harmVoltage;

// Her cihaz kendi MAC adresinden benzersiz bir ID uretir -- ayni firmware
// tum cihazlara (marka farketmeksizin) degistirilmeden yuklenebilir, fabrikada
// cihaza ozel ayar gerekmez. Topic'ler bu ID'ye gore olusturulur.
String deviceId;
String liveTopic, energyTopic, statusTopic, cmdTopic, otaTopic;
String wifiTopic;   // WiFi degistirme sonucunun bildirildigi konu
// 0 = bekleyen sonuc yok, 1 = yeni aga gecildi, 2 = gecilemedi, eskiye donuldu.
// Sonuc HEMEN yayinlanamiyor: degisim sirasinda MQTT baglantisi kopuyor.
int wifiDegisimSonucu = 0;
String statsTopic, peaksTopic, demandTopic, harmonicsTopic, infoTopic;

// OTA guvenlik agi: yeni firmware yazildiktan hemen sonra "pending" olarak
// isaretlenir. Bu firmware WiFi+MQTT'ye baglanip en az bir kez canli veri
// okuyabilirse "onaylanmis" sayilir (esp_ota_mark_app_valid_cancel_rollback).
// Belirlenen surede onaylanamazsa onceki (calisir durumdaki) firmware'e
// otomatik geri donulur -- kotu bir OTA cihazi sahada "tugla"ya cevirmesin diye.
Preferences otaPrefs;
bool otaPendingConfirm = false;
unsigned long otaPendingSince = 0;
#define OTA_CONFIRM_TIMEOUT_MS (90UL * 1000UL)

// WiFi kurulumu: telefon uygulamasi (Tuya benzeri) cihazin actigi AP'ye baglanip
// buradan ev WiFi bilgisini gonderir -- tarayici/portal sayfasi yok, tamamen JSON.
Preferences prefs;
WebServer provisionServer(80);
volatile bool wifiConfigured = false;
String pendingSsid, pendingPass;

void setupDeviceId() {
  // esp_read_mac() donanima gomulu MAC'i dogrudan okur (efuse) -- WiFi.macAddress()
  // aksine WiFi radyosunun baslatilmis/bagli olmasina bagli degildir, bu yuzden
  // WiFi baglanmadan ONCE bile guvenilir sonuc verir.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char idBuf[20];
  snprintf(idBuf, sizeof(idBuf), "anl21-%02x%02x%02x", mac[3], mac[4], mac[5]);
  deviceId = String(idBuf);
  liveTopic   = "powermeter/" + deviceId + "/live";
  energyTopic = "powermeter/" + deviceId + "/energy";
  statusTopic = "powermeter/" + deviceId + "/status";
  cmdTopic    = "powermeter/" + deviceId + "/cmd";
  otaTopic    = "powermeter/" + deviceId + "/ota";
  statsTopic     = "powermeter/" + deviceId + "/stats";
  peaksTopic     = "powermeter/" + deviceId + "/peaks";
  demandTopic    = "powermeter/" + deviceId + "/demand";
  harmonicsTopic = "powermeter/" + deviceId + "/harmonics";
  wifiTopic = "powermeter/" + deviceId + "/wifi";
  infoTopic      = "powermeter/" + deviceId + "/info";
}

void preTransmission()  { digitalWrite(RE_DE_PIN, HIGH); }
void postTransmission() { digitalWrite(RE_DE_PIN, LOW); }

// Excel haritasindaki "Signed 32" degerler, buffer'da 2 ardisik 16-bit register olarak
// geliyor. Isaretli (negatif olabilen) 32-bit'e ceviriyoruz.
int32_t readS32(uint16_t bufIndex) {
  uint16_t a = node.getResponseBuffer(bufIndex);
  uint16_t b = node.getResponseBuffer(bufIndex + 1);
  uint16_t hi = WORDS_HIGH_FIRST ? a : b;
  uint16_t lo = WORDS_HIGH_FIRST ? b : a;
  return (int32_t)(((uint32_t)hi << 16) | lo);
}

// Anlik degerler bloğu: 10050 (1. Faz Notr Gerilim) - 10125 (3. Faz THID) arasi,
// tumu Signed 32 (2 register), toplam 76 register. ModbusMaster kutuphanesinin
// dahili yanit tamponu SABIT 64 register (ku8MaxBufferSize, ModbusMaster.h) --
// tek seferde 76 register istemek 64'ten sonraki her seyi (thvd1-3, thd1-3)
// sessizce 0xFFFF (gecersiz/-1) olarak donduruyordu, gercek "yuk yok" durumuyla
// ayirt edilemiyordu. Bu yuzden 64 register sinirinin altinda kalan iki parcali
// okumaya bolundu: ilk parca pf3'e kadar (tam 64 register), ikinci parca thvd/thd.
#define LIVE_BLOCK1_START 10050

bool readPowerAnalyzer() {
  uint8_t result = node.readHoldingRegisters(LIVE_BLOCK1_START, 64); // 10050-10113
  if (result != node.ku8MBSuccess) return false;

  #define L1(off) readS32(off)
  data.v1   = L1(10050 - LIVE_BLOCK1_START) * 0.1;
  data.v2   = L1(10052 - LIVE_BLOCK1_START) * 0.1;
  data.v3   = L1(10054 - LIVE_BLOCK1_START) * 0.1;
  data.vN   = L1(10056 - LIVE_BLOCK1_START) * 0.1;
  data.vL12 = L1(10058 - LIVE_BLOCK1_START) * 0.1;
  data.vL23 = L1(10060 - LIVE_BLOCK1_START) * 0.1;
  data.vL31 = L1(10062 - LIVE_BLOCK1_START) * 0.1;

  data.i1 = L1(10064 - LIVE_BLOCK1_START) * 0.001;
  data.i2 = L1(10066 - LIVE_BLOCK1_START) * 0.001;
  data.i3 = L1(10068 - LIVE_BLOCK1_START) * 0.001;
  data.iN = L1(10070 - LIVE_BLOCK1_START) * 0.001;

  data.p1 = L1(10072 - LIVE_BLOCK1_START) * 1.0;
  data.p2 = L1(10074 - LIVE_BLOCK1_START) * 1.0;
  data.p3 = L1(10076 - LIVE_BLOCK1_START) * 1.0;

  data.q1 = L1(10078 - LIVE_BLOCK1_START) * 1.0;
  data.q2 = L1(10080 - LIVE_BLOCK1_START) * 1.0;
  data.q3 = L1(10082 - LIVE_BLOCK1_START) * 1.0;

  data.s1 = L1(10084 - LIVE_BLOCK1_START) * 1.0;
  data.s2 = L1(10086 - LIVE_BLOCK1_START) * 1.0;
  data.s3 = L1(10088 - LIVE_BLOCK1_START) * 1.0;

  data.f1 = L1(10090 - LIVE_BLOCK1_START) * 0.01;
  data.f2 = L1(10092 - LIVE_BLOCK1_START) * 0.01;
  data.f3 = L1(10094 - LIVE_BLOCK1_START) * 0.01;

  data.cos1 = L1(10096 - LIVE_BLOCK1_START) * 0.001;
  data.cos2 = L1(10098 - LIVE_BLOCK1_START) * 0.001;
  data.cos3 = L1(10100 - LIVE_BLOCK1_START) * 0.001;

  data.pf1 = L1(10108 - LIVE_BLOCK1_START) * 0.001;
  data.pf2 = L1(10110 - LIVE_BLOCK1_START) * 0.001;
  data.pf3 = L1(10112 - LIVE_BLOCK1_START) * 0.001;
  #undef L1

  delay(30);
  result = node.readHoldingRegisters(10114, 12); // 10114-10125 (thvd1-3, thd1-3)
  if (result != node.ku8MBSuccess) return false;

  #define L2(off) readS32(off)
  // Bu cihaz THD'yi gerilim (THVD) ve akim (THID) olarak ayri veriyor -- ikisi de
  // gonderiliyor: "thd" alani akim THD'si (THID), "thvd" alani gerilim THD'si.
  data.thvd1 = L2(10114 - 10114) * 0.1;
  data.thvd2 = L2(10116 - 10114) * 0.1;
  data.thvd3 = L2(10118 - 10114) * 0.1;

  data.thd1 = L2(10120 - 10114) * 0.1;
  data.thd2 = L2(10122 - 10114) * 0.1;
  data.thd3 = L2(10124 - 10114) * 0.1;
  #undef L2

  return true;
}

// Enerji sayaclari 64-bit (4 register) geliyor ama Wh cinsinden gercekci degerler
// (yillar boyu birikse bile) rahatlikla 32-bit'e sigar -- ANL13 ile ayni sekilde
// 32-bit'e indirgeyerek tasiyoruz (JSON'da %lu, ESP32'de 64-bit printf sikintili).
// En anlamli 2 register (LSW ciftini) aliyoruz, ust 2 register (hep 0 olmasi beklenir) atlanir.
uint32_t readEnergyU32(uint16_t bufIndex) {
  uint16_t w0 = node.getResponseBuffer(bufIndex);
  uint16_t w1 = node.getResponseBuffer(bufIndex + 1);
  uint16_t w2 = node.getResponseBuffer(bufIndex + 2);
  uint16_t w3 = node.getResponseBuffer(bufIndex + 3);
  // WORDS_HIGH_FIRST true ise sira MSW->LSW (w0..w3), LSW cifti w2,w3'tur.
  uint16_t hi = WORDS_HIGH_FIRST ? w2 : w1;
  uint16_t lo = WORDS_HIGH_FIRST ? w3 : w0;
  return ((uint32_t)hi << 16) | lo;
}

// Kaynak: "Enerjiler" sayfasi -- Tuketim toplam blogu 13020, Uretim toplam blogu 13280.
bool readEnergyBlock() {
  uint8_t result = node.readHoldingRegisters(13020, 12); // Toplam Aktif/Endüktif/Kapasitif (Tüketim)
  if (result != node.ku8MBSuccess) return false;
  energy.activeWhTuketim      = readEnergyU32(0);
  energy.inductiveVArhTuketim = readEnergyU32(4);
  energy.capacitiveVArhTuketim= readEnergyU32(8);

  delay(30); // RS-485 hattina ardisik istekler arasi toparlanma suresi
  result = node.readHoldingRegisters(13280, 12); // Toplam Aktif/Endüktif/Kapasitif (Üretim)
  if (result != node.ku8MBSuccess) return false;
  energy.activeWhUretim      = readEnergyU32(0);
  energy.inductiveVArhUretim = readEnergyU32(4);
  energy.capacitiveVArhUretim= readEnergyU32(8);

  return true;
}

// Register 221 = Akim Trafo Orani (Table Index), Parametreler sayfasi, W/R, 0-69.
// Deger 0-69 arasi bir INDEKS'tir, gercek orana (X/5 A) cevirisi backend'de yapilir.
bool readCtRatio() {
  uint8_t result = node.readHoldingRegisters(221, 1);
  if (result != node.ku8MBSuccess) return false;
  ctRatio = node.getResponseBuffer(0);
  return true;
}

// ==================== Toplam ve Ortalama Degerler (30sn) ====================
// Kaynak: "Toplam ve Ortalama Değerler" sayfasi, 10410-10500 araligi tek okuma.
struct TotalsData {
  float pActiveImp, pReactiveImp, pInductiveImp, pCapacitiveImp, pApparentImp;
  float pActiveExp, pReactiveExp, pInductiveExp, pCapacitiveExp, pApparentExp;
  float avgCurrentImp, avgActivePowerImp, avgCosImp, avgTanImp, avgPfImp;
  float avgCurrentExp, avgActivePowerExp, avgCosExp, avgTanExp, avgPfExp;
  float avgVoltageLN, avgVoltageLL, avgFrequency, avgThid, avgThvd;
};
TotalsData totals;
#define TOTALS_START 10410
// ModbusMaster'in 64 registerlik sabit tampon siniri (ku8MaxBufferSize) yuzunden
// 92 registerlik tek okuma bolundu: ilk parca tam 64 register (avgPfExp'e kadar,
// dogal bir alan sinirinda), ikinci parca kalan 28 register.
#define TOTALS1_COUNT 64
#define TOTALS2_START 10474

bool readTotals() {
  uint8_t result = node.readHoldingRegisters(TOTALS_START, TOTALS1_COUNT); // 10410-10473
  if (result != node.ku8MBSuccess) return false;
  #define T1(addr) readS32(addr - TOTALS_START)
  totals.pActiveImp     = T1(10410) * 1.0;
  totals.pReactiveImp   = T1(10412) * 1.0;
  totals.pInductiveImp  = T1(10414) * 1.0;
  totals.pCapacitiveImp = T1(10416) * 1.0;
  totals.pApparentImp   = T1(10418) * 1.0;
  totals.pActiveExp     = T1(10430) * 1.0;
  totals.pReactiveExp   = T1(10432) * 1.0;
  totals.pInductiveExp  = T1(10434) * 1.0;
  totals.pCapacitiveExp = T1(10436) * 1.0;
  totals.pApparentExp   = T1(10438) * 1.0;
  totals.avgCurrentImp     = T1(10452) * 0.001;
  totals.avgActivePowerImp = T1(10454) * 1.0;
  totals.avgCosImp         = T1(10456) * 0.001;
  totals.avgTanImp         = T1(10458) * 0.1;
  // "Ortalama Pf" icin carpan Excel'de belirtilmemis ("-") -- ayni sayfadaki
  // cos/tan alanlarina benzer sekilde 0.001 varsayildi, ilk gercek okumada
  // deger 0-1 araliginda mantikli gorunmezse duzeltilecek.
  totals.avgPfImp = T1(10460) * 0.001;
  totals.avgCurrentExp     = T1(10472) * 0.001;
  #undef T1

  delay(30);
  result = node.readHoldingRegisters(TOTALS2_START, 28); // 10474-10501
  if (result != node.ku8MBSuccess) return false;
  #define T2(addr) readS32(addr - TOTALS2_START)
  totals.avgActivePowerExp = T2(10474) * 1.0;
  totals.avgCosExp         = T2(10476) * 0.001;
  totals.avgTanExp         = T2(10478) * 0.1;
  totals.avgPfExp = T2(10480) * 0.001;
  totals.avgVoltageLN = T2(10492) * 0.1;
  totals.avgVoltageLL = T2(10494) * 0.1;
  totals.avgFrequency = T2(10496) * 0.01;
  totals.avgThid = T2(10498) * 0.1;
  totals.avgThvd = T2(10500) * 0.1;
  #undef T2
  return true;
}

// ==================== Min/Max Tepe Degerleri (60sn) ====================
// base: blogun ilk adresi (Tuketim=11010, Uretim=11260). Uc parcali okuma
// gerekiyor: (1) ModbusMaster'in sabit 64 registerlik tampon siniri (ku8MaxBufferSize,
// ModbusMaster.h) V/I/Guc blogunun 80 register olmasi nedeniyle tek okumaya
// sigmiyor -- 64'ten sonrasi sessizce 0xFFFF/gecersiz veri donuyordu (gercek
// hatayla ayirt edilemeyen bir bug, canli veride de ayni sorun bulunup duzeltildi).
// (2) 11090-11099 ve 11124-11135 arasi bos/rezerve, okunmasina gerek yok.
void logPeakRead(const char *label, uint16_t addr, uint16_t count, uint8_t result) {
  Serial.print("PeakRead "); Serial.print(label);
  Serial.print(" addr="); Serial.print(addr);
  Serial.print(" count="); Serial.print(count);
  Serial.print(" result="); Serial.print(result);
  Serial.print(" buf0="); Serial.print(node.getResponseBuffer(0), HEX);
  Serial.print(" buf1="); Serial.println(node.getResponseBuffer(1), HEX);
}

// Ardisik cok sayida Tepe/Demand okumasindan sonra bazen "basarili" (result=0)
// donup sabit/anlamsiz bir deger (buf0=0xA000) donen bir sorun gozlendi --
// clearResponseBuffer() ile eski tampon verisi olmadigi dogrulandi, artan
// bekleme suresi de cozmedi. Bilinen bu imzayi (0xA000) gordugunde okumayi
// birkac kez tekrar dener; boylece kok neden tam netlesmeden de veri kurtarilir.
#define PEAK_BAD_SIGNATURE 0xA000

bool readHoldingRegistersRetry(const char *label, uint16_t addr, uint16_t count, int maxAttempts = 3) {
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    node.clearResponseBuffer();
    uint8_t result = node.readHoldingRegisters(addr, count);
    uint16_t buf0 = node.getResponseBuffer(0);
    logPeakRead(label, addr, count, result);
    if (result == node.ku8MBSuccess && buf0 != PEAK_BAD_SIGNATURE) {
      return true;
    }
    delay(150);
  }
  return false;
}

bool readPeakBlock(PeakBlock &out, uint16_t base) {
  if (!readHoldingRegistersRetry("sub1a", base, 64)) return false; // base .. base+63 (V/I/Guc, 1. parca)
  #define P1(off) readS32(off)
  out.minVln1 = P1(0) * 0.1;   out.minVln2 = P1(2) * 0.1;   out.minVln3 = P1(4) * 0.1;
  out.minVn   = P1(6) * 0.1;
  out.maxVln1 = P1(8) * 0.1;   out.maxVln2 = P1(10) * 0.1;  out.maxVln3 = P1(12) * 0.1;
  out.maxVn   = P1(14) * 0.1;
  out.minVll1 = P1(16) * 0.1;  out.minVll2 = P1(18) * 0.1;  out.minVll3 = P1(20) * 0.1;
  out.maxVll1 = P1(22) * 0.1;  out.maxVll2 = P1(24) * 0.1;  out.maxVll3 = P1(26) * 0.1;
  out.minI1 = P1(28) * 0.001;  out.minI2 = P1(30) * 0.001;  out.minI3 = P1(32) * 0.001;
  out.minIn = P1(34) * 0.001;
  out.maxI1 = P1(36) * 0.001;  out.maxI2 = P1(38) * 0.001;  out.maxI3 = P1(40) * 0.001;
  out.maxIn = P1(42) * 0.001;
  out.minP1 = P1(44); out.minP2 = P1(46); out.minP3 = P1(48);
  out.maxP1 = P1(50); out.maxP2 = P1(52); out.maxP3 = P1(54);
  out.minQ1 = P1(56); out.minQ2 = P1(58); out.minQ3 = P1(60);
  out.maxQ1 = P1(62);
  #undef P1

  if (!readHoldingRegistersRetry("sub1b", base + 64, 16)) return false; // base+64 .. base+79 (V/I/Guc, 2. parca)
  #define P1B(off) readS32(off)
  out.maxQ2 = P1B(0); out.maxQ3 = P1B(2);
  out.minS1 = P1B(4); out.minS2 = P1B(6); out.minS3 = P1B(8);
  out.maxS1 = P1B(10); out.maxS2 = P1B(12); out.maxS3 = P1B(14);
  #undef P1B

  // 48 register (47 degil): son alan "Max Akim Deng.(%)" 2 register (11146-11147)
  // kapliyor, 47 istersek buffer'in son register'ini okumadan biraktigimiz icin
  // maxIUnbal ceride kalan (okunmamis) buffer verisiyle hesaplanirdi.
  if (!readHoldingRegistersRetry("sub2", base + 90, 48)) return false; // THVD/THID/Frekans/Dengesizlik
  #define P2(off) readS32(off)
  out.minThvd1 = P2(0) * 0.1;  out.minThvd2 = P2(2) * 0.1;  out.minThvd3 = P2(4) * 0.1;
  out.maxThvd1 = P2(6) * 0.1;  out.maxThvd2 = P2(8) * 0.1;  out.maxThvd3 = P2(10) * 0.1;
  out.minThid1 = P2(12) * 0.1; out.minThid2 = P2(14) * 0.1; out.minThid3 = P2(16) * 0.1;
  out.maxThid1 = P2(18) * 0.1; out.maxThid2 = P2(20) * 0.1; out.maxThid3 = P2(22) * 0.1;
  out.minFreq = P2(36) * 0.01; out.maxFreq = P2(38) * 0.01;
  out.minVUnbal = P2(40); out.maxVUnbal = P2(42);
  out.minIUnbal = P2(44); out.maxIUnbal = P2(46);
  #undef P2
  return true;
}

// ==================== Demand Degerleri (60sn) ====================
// Kaynak: "Demandlar" sayfasi, ayni Tuketim/Uretim iki-blok deseni.
bool readDemandBlock(DemandBlock &out, uint16_t base) {
  if (!readHoldingRegistersRetry("demand1", base, 60)) return false; // V/I/P/Q/S demand
  #define D1(off) readS32(off)
  out.maxDV1 = D1(0) * 0.1;  out.maxDV2 = D1(2) * 0.1;  out.maxDV3 = D1(4) * 0.1;
  out.minDV1 = D1(6) * 0.1;  out.minDV2 = D1(8) * 0.1;  out.minDV3 = D1(10) * 0.1;
  out.maxDI1 = D1(12) * 0.001; out.maxDI2 = D1(14) * 0.001; out.maxDI3 = D1(16) * 0.001;
  out.minDI1 = D1(18) * 0.001; out.minDI2 = D1(20) * 0.001; out.minDI3 = D1(22) * 0.001;
  out.maxDP1 = D1(24); out.maxDP2 = D1(26); out.maxDP3 = D1(28);
  out.minDP1 = D1(30); out.minDP2 = D1(32); out.minDP3 = D1(34);
  out.maxDQ1 = D1(36); out.maxDQ2 = D1(38); out.maxDQ3 = D1(40);
  out.minDQ1 = D1(42); out.minDQ2 = D1(44); out.minDQ3 = D1(46);
  out.maxDS1 = D1(48); out.maxDS2 = D1(50); out.maxDS3 = D1(52);
  out.minDS1 = D1(54); out.minDS2 = D1(56); out.minDS3 = D1(58);
  #undef D1

  if (!readHoldingRegistersRetry("demand2", base + 90, 24)) return false; // THVD/THID demand
  #define D2(off) readS32(off)
  out.maxDThvd1 = D2(0) * 0.1;  out.maxDThvd2 = D2(2) * 0.1;  out.maxDThvd3 = D2(4) * 0.1;
  out.minDThvd1 = D2(6) * 0.1;  out.minDThvd2 = D2(8) * 0.1;  out.minDThvd3 = D2(10) * 0.1;
  out.maxDThid1 = D2(12) * 0.1; out.maxDThid2 = D2(14) * 0.1; out.maxDThid3 = D2(16) * 0.1;
  out.minDThid1 = D2(18) * 0.1; out.minDThid2 = D2(20) * 0.1; out.minDThid3 = D2(22) * 0.1;
  #undef D2
  return true;
}

// ==================== Harmonikler (5 dk) ====================
// Kaynak: "Akım/Gerilim Harmonikleri" sayfalari. Bu sayfalar 62. harmonige kadar
// gidiyor ama 33. harmonikten sonra register genisligi 16-bit'ten 32-bit'e
// degisiyor (kilavuzda acik degil) -- v1'de sadece guvenilir/tek-genislikli
// blogu (3.-31. tek harmonikler, hepsi Signed 16) okuyoruz. Bu, guc kalitesi
// raporlamasinda (IEEE 519 vb.) yaygin kullanilan araligi zaten kapsiyor.
bool readHarmonicsBlock(HarmonicsBlock &out, uint16_t base) {
  uint8_t result = node.readHoldingRegisters(base, 48); // THD(3) + 15 harmonik x 3 faz
  if (result != node.ku8MBSuccess) return false;
  out.thd1 = node.getResponseBuffer(0) * 0.1;
  out.thd2 = node.getResponseBuffer(1) * 0.1;
  out.thd3 = node.getResponseBuffer(2) * 0.1;
  for (int h = 0; h < HARMONIC_COUNT; h++) {
    for (int ph = 0; ph < 3; ph++) {
      out.h[h][ph] = node.getResponseBuffer(3 + h * 3 + ph) * 0.1;
    }
  }
  return true;
}

// ==================== Cihaz Bilgileri (sadece aciliste bir kez) ====================
// Kaynak: "Özel" sayfasi, 100-125 araligi. Carpan yok, ham 32-bit degerler.
struct DeviceInfo {
  int32_t seriNo, urunId, kartId, sistemVersiyon, ulkeKodu, firmaKodu;
  int32_t beslemeTipi, ekranTipi, keyboardTipi, kutuTipi, klemensTipi, connections, storage;
};
DeviceInfo deviceInfo;
bool deviceInfoOk = false;
bool deviceInfoPublished = false;

bool readDeviceInfo() {
  uint8_t result = node.readHoldingRegisters(100, 26);
  if (result != node.ku8MBSuccess) return false;
  #define I(off) readS32(off)
  deviceInfo.seriNo         = I(0);
  deviceInfo.urunId         = I(2);
  deviceInfo.kartId         = I(4);
  deviceInfo.sistemVersiyon = I(6);
  deviceInfo.ulkeKodu       = I(8);
  deviceInfo.firmaKodu      = I(10);
  deviceInfo.beslemeTipi    = I(12);
  deviceInfo.ekranTipi      = I(14);
  deviceInfo.keyboardTipi   = I(16);
  deviceInfo.kutuTipi       = I(18);
  deviceInfo.klemensTipi    = I(20);
  deviceInfo.connections    = I(22);
  deviceInfo.storage        = I(24);
  #undef I
  return true;
}

// Backend'ten kendi cmd topic'imize gelen {"register": 9001, "value": 43605} gibi
// bir komutu ilgili registera yazar (fonksiyon kodu 06, single register write).
// "value" verilmezse varsayilan 1'dir. Excel'deki "Komutlar" sayfasi 0xAA55
// (43605) tetik degerini dokumante ediyor -- ANL13'te bu bilinmiyordu.
// Basit JSON string alan cikarici -- {"url":"...","version":"..."} gibi duz
// bir govdeden bir alani cikarir. WiFi provisioning'deki extractJsonString ile
// ayni yaklasim, burada da ArduinoJson bagimliligi eklemekten kaciniliyor.
bool extractJsonField(const char* body, const char* key, char* out, size_t outSize) {
  char pattern[32];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(body, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  p++;
  const char* end = strchr(p, '"');
  if (!end) return false;
  size_t len = end - p;
  if (len >= outSize) len = outSize - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

void handleOtaMessage(byte* payload, unsigned int length) {
  char buf[400];
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);
  buf[n] = '\0';

  char url[300];
  char version[32];
  if (!extractJsonField(buf, "url", url, sizeof(url))) {
    Serial.println("OTA: gecersiz mesaj, url alani yok");
    return;
  }
  extractJsonField(buf, "version", version, sizeof(version));

  Serial.print("OTA baslatiliyor, surum: ");
  Serial.print(version);
  Serial.print(" url: ");
  Serial.println(url);

  // Yazma basladiktan sonra bir kesinti/cokme olursa bir dahaki acilista bunu
  // hatirlayip geri donus zaman asimini calistirabilmek icin flash'a yaziliyor.
  otaPrefs.putBool("pending", true);

  WiFiClientSecure client;
  // OTA dogrulanmis TLS kullanir. setInsecure() ile araya giren biri cihaza
  // istedigi firmware'i yukleyebilirdi -- telemetriyi okumaktan daha agir bir
  // acikti. Ayni emniyet yedegi burada da gecerli.
  if (tlsDogrulamaKapali) {
    client.setInsecure();
  } else {
    client.setCACert(ISRG_ROOTS);
  }
  httpUpdate.rebootOnUpdate(false);
  t_httpUpdate_return ret = httpUpdate.update(client, url);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("OTA basarisiz (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      otaPrefs.putBool("pending", false); // yazma hic basarili olmadi, eski firmware zaten aktif
      mqttClient.publish(statusTopic.c_str(), "ota_failed", false);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("OTA: guncelleme yok");
      otaPrefs.putBool("pending", false);
      break;
    case HTTP_UPDATE_OK:
      Serial.println("OTA basarili, yeniden baslatiliyor...");
      mqttClient.publish(statusTopic.c_str(), "ota_rebooting", false);
      delay(200);
      ESP.restart();
      break;
  }
}

// Yeni yazilan firmware WiFi+MQTT'ye baglanip en az bir kez canli veri
// okuyabildiginde cagrilir -- bu surumu "gecerli" isaretler, bir sonraki
// acilista rollback tetiklenmez.
void confirmOtaIfPending() {
  if (!otaPendingConfirm) return;
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    Serial.println("OTA onaylandi, bu surum kalici.");
  } else {
    Serial.printf("OTA onayi basarisiz (esp_err=%d) -- rollback bu cihazda desteklenmiyor olabilir.\n", err);
  }
  otaPrefs.putBool("pending", false);
  otaPendingConfirm = false;
}

// Acilista "pending" isaretliyse (bir onceki OTA hic onaylanmamis) belli bir
// sure icinde onaylanmazsa onceki firmware'e geri donulur.
void checkOtaRollbackTimeout() {
  if (!otaPendingConfirm) return;
  if (millis() - otaPendingSince < OTA_CONFIRM_TIMEOUT_MS) return;
  Serial.println("OTA onaylanamadi, onceki firmware'e donuluyor...");
  esp_ota_mark_app_invalid_rollback_and_reboot();
  // Rollback desteklenmiyorsa yukarasi geri doner -- sonsuz dongude tekrar
  // denemeyelim diye pending'i temizleyip normal calismaya devam ediyoruz.
  otaPrefs.putBool("pending", false);
  otaPendingConfirm = false;
}

// Kucuk bir JSON string cikarici. ArduinoJson bagimliligi eklememek icin --
// tek ihtiyacimiz iki alan okumak.
static bool jsonDegerCek(const char* buf, const char* anahtar, String &cikti) {
  char desen[24];
  snprintf(desen, sizeof(desen), "\"%s\"", anahtar);
  const char* p = strstr(buf, desen);
  if (!p) return false;
  p = strchr(p + strlen(desen), ':');
  if (!p) return false;
  p = strchr(p, '"');            // degerin acilis tirnagi
  if (!p) return false;
  p++;
  const char* son = p;
  // Kacisli tirnak (\") degerin sonu degildir -- WiFi parolalarinda olabilir.
  while (*son && !(*son == '"' && *(son - 1) != '\\')) son++;
  if (!*son) return false;
  cikti = "";
  for (const char* q = p; q < son; q++) cikti += *q;
  return true;
}


// Cihazi baska bir WiFi agina tasir.
//
// EN KRITIK NOKTA: yanlis bilgi girilirse cihaz tamamen kaybolur ve yerinde
// mudahale gerekir. Bu yuzden eski bilgiler saklanip yeni ag DENENIYOR; olmazsa
// eskiye donuluyor. Yeni bilgiler ancak baglanti KURULDUKTAN sonra kaydediliyor.
void handleWifiConfig(const char* buf) {
  String yeniSsid, yeniPass;
  if (!jsonDegerCek(buf, "ssid", yeniSsid) || yeniSsid.length() == 0) {
    Serial.println("WiFi komutu: ssid yok, yoksayildi.");
    return;
  }
  jsonDegerCek(buf, "pass", yeniPass);   // acik ag icin bos olabilir

  String eskiSsid = prefs.getString("ssid", "");
  String eskiPass = prefs.getString("pass", "");

  Serial.print("WiFi degistirme istegi: ");
  Serial.println(yeniSsid);

  mqttClient.disconnect();
  WiFi.disconnect();
  delay(300);

  if (connectToWifi(yeniSsid, yeniPass, 20000)) {
    // Yalnizca BASARILI baglantidan sonra kaydet.
    prefs.putString("ssid", yeniSsid);
    prefs.putString("pass", yeniPass);
    Serial.print("Yeni aga gecildi: ");
    Serial.println(WiFi.SSID());
    wifiDegisimSonucu = 1;
  } else {
    Serial.println("Yeni aga baglanilamadi, eski aga donuluyor.");
    if (connectToWifi(eskiSsid, eskiPass, 20000)) {
      Serial.println("Eski aga donuldu, kayitli bilgi degismedi.");
    } else {
      // Iki ag da yoksa loop() 3 dakika sonra yeniden baslatir ve acilista
      // kurulum moduna dusulur -- kayitli bilgi hala ESKISI oldugu icin,
      // eski ag geri gelirse cihaz kendiliginden toparlanir.
      Serial.println("Eski aga da donulemedi -- yeniden baslatma bekleniyor.");
    }
    wifiDegisimSonucu = 2;
  }
}


void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (otaTopic.length() && strcmp(topic, otaTopic.c_str()) == 0) {
    handleOtaMessage(payload, length);
    return;
  }

  // 64 bayt yalnizca {"register":N,"value":M} icin yetiyordu; WiFi komutunda
  // SSID + parola var, daha genis tampon gerekiyor.
  char buf[256];
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);
  buf[n] = '\0';

  if (strstr(buf, "\"cmd\"") && strstr(buf, "wifi")) {
    handleWifiConfig(buf);
    return;
  }

  char* p = strstr(buf, "\"register\"");
  if (!p) return;
  p = strchr(p, ':');
  if (!p) return;
  int reg = atoi(p + 1);
  if (reg <= 0) return;

  int value = 1;
  char* vp = strstr(buf, "\"value\"");
  if (vp) {
    vp = strchr(vp, ':');
    if (vp) value = atoi(vp + 1);
  }

  // Mesaj tam main loop'un Modbus istegi ortasindayken gelirse cakisip HATA
  // verebiliyor -- kisa bekleme ile birkac kez yeniden dene.
  uint8_t result = 1;
  int deneme = 0;
  for (deneme = 1; deneme <= 3; deneme++) {
    result = node.writeSingleRegister(reg, value);
    if (result == node.ku8MBSuccess) break;
    delay(60);
  }
  Serial.print("Komut yazildi, register=");
  Serial.print(reg);
  Serial.print(" value=");
  Serial.print(value);
  Serial.print(" deneme=");
  Serial.print(deneme <= 3 ? deneme : 3);
  Serial.print(" sonuc=");
  Serial.println(result == node.ku8MBSuccess ? "OK" : "HATA");
}

// ==================== JSON payload olusturucular ====================

void buildPeakJson(char *buf, size_t bufSize, const PeakBlock &b) {
  snprintf(buf, bufSize,
    "{\"minVln1\":%.1f,\"minVln2\":%.1f,\"minVln3\":%.1f,\"minVn\":%.1f,"
    "\"maxVln1\":%.1f,\"maxVln2\":%.1f,\"maxVln3\":%.1f,\"maxVn\":%.1f,"
    "\"minVll1\":%.1f,\"minVll2\":%.1f,\"minVll3\":%.1f,"
    "\"maxVll1\":%.1f,\"maxVll2\":%.1f,\"maxVll3\":%.1f,"
    "\"minI1\":%.3f,\"minI2\":%.3f,\"minI3\":%.3f,\"minIn\":%.3f,"
    "\"maxI1\":%.3f,\"maxI2\":%.3f,\"maxI3\":%.3f,\"maxIn\":%.3f,"
    "\"minP1\":%.0f,\"minP2\":%.0f,\"minP3\":%.0f,\"maxP1\":%.0f,\"maxP2\":%.0f,\"maxP3\":%.0f,"
    "\"minQ1\":%.0f,\"minQ2\":%.0f,\"minQ3\":%.0f,\"maxQ1\":%.0f,\"maxQ2\":%.0f,\"maxQ3\":%.0f,"
    "\"minS1\":%.0f,\"minS2\":%.0f,\"minS3\":%.0f,\"maxS1\":%.0f,\"maxS2\":%.0f,\"maxS3\":%.0f,"
    "\"minThvd1\":%.1f,\"minThvd2\":%.1f,\"minThvd3\":%.1f,\"maxThvd1\":%.1f,\"maxThvd2\":%.1f,\"maxThvd3\":%.1f,"
    "\"minThid1\":%.1f,\"minThid2\":%.1f,\"minThid3\":%.1f,\"maxThid1\":%.1f,\"maxThid2\":%.1f,\"maxThid3\":%.1f,"
    "\"minFreq\":%.2f,\"maxFreq\":%.2f,\"minVUnbal\":%.1f,\"maxVUnbal\":%.1f,\"minIUnbal\":%.1f,\"maxIUnbal\":%.1f}",
    b.minVln1, b.minVln2, b.minVln3, b.minVn, b.maxVln1, b.maxVln2, b.maxVln3, b.maxVn,
    b.minVll1, b.minVll2, b.minVll3, b.maxVll1, b.maxVll2, b.maxVll3,
    b.minI1, b.minI2, b.minI3, b.minIn, b.maxI1, b.maxI2, b.maxI3, b.maxIn,
    b.minP1, b.minP2, b.minP3, b.maxP1, b.maxP2, b.maxP3,
    b.minQ1, b.minQ2, b.minQ3, b.maxQ1, b.maxQ2, b.maxQ3,
    b.minS1, b.minS2, b.minS3, b.maxS1, b.maxS2, b.maxS3,
    b.minThvd1, b.minThvd2, b.minThvd3, b.maxThvd1, b.maxThvd2, b.maxThvd3,
    b.minThid1, b.minThid2, b.minThid3, b.maxThid1, b.maxThid2, b.maxThid3,
    b.minFreq, b.maxFreq, b.minVUnbal, b.maxVUnbal, b.minIUnbal, b.maxIUnbal);
}

void buildDemandJson(char *buf, size_t bufSize, const DemandBlock &d) {
  snprintf(buf, bufSize,
    "{\"maxDV1\":%.1f,\"maxDV2\":%.1f,\"maxDV3\":%.1f,\"minDV1\":%.1f,\"minDV2\":%.1f,\"minDV3\":%.1f,"
    "\"maxDI1\":%.3f,\"maxDI2\":%.3f,\"maxDI3\":%.3f,\"minDI1\":%.3f,\"minDI2\":%.3f,\"minDI3\":%.3f,"
    "\"maxDP1\":%.0f,\"maxDP2\":%.0f,\"maxDP3\":%.0f,\"minDP1\":%.0f,\"minDP2\":%.0f,\"minDP3\":%.0f,"
    "\"maxDQ1\":%.0f,\"maxDQ2\":%.0f,\"maxDQ3\":%.0f,\"minDQ1\":%.0f,\"minDQ2\":%.0f,\"minDQ3\":%.0f,"
    "\"maxDS1\":%.0f,\"maxDS2\":%.0f,\"maxDS3\":%.0f,\"minDS1\":%.0f,\"minDS2\":%.0f,\"minDS3\":%.0f,"
    "\"maxDThvd1\":%.1f,\"maxDThvd2\":%.1f,\"maxDThvd3\":%.1f,\"minDThvd1\":%.1f,\"minDThvd2\":%.1f,\"minDThvd3\":%.1f,"
    "\"maxDThid1\":%.1f,\"maxDThid2\":%.1f,\"maxDThid3\":%.1f,\"minDThid1\":%.1f,\"minDThid2\":%.1f,\"minDThid3\":%.1f}",
    d.maxDV1, d.maxDV2, d.maxDV3, d.minDV1, d.minDV2, d.minDV3,
    d.maxDI1, d.maxDI2, d.maxDI3, d.minDI1, d.minDI2, d.minDI3,
    d.maxDP1, d.maxDP2, d.maxDP3, d.minDP1, d.minDP2, d.minDP3,
    d.maxDQ1, d.maxDQ2, d.maxDQ3, d.minDQ1, d.minDQ2, d.minDQ3,
    d.maxDS1, d.maxDS2, d.maxDS3, d.minDS1, d.minDS2, d.minDS3,
    d.maxDThvd1, d.maxDThvd2, d.maxDThvd3, d.minDThvd1, d.minDThvd2, d.minDThvd3,
    d.maxDThid1, d.maxDThid2, d.maxDThid3, d.minDThid1, d.minDThid2, d.minDThid3);
}

// Harmonik dizisini "h3_l1":x,"h3_l2":x,... seklinde JSON'a ekler, kalan buffer
// alanini ve yazilan uzunlugu dondurur.
size_t appendHarmonics(char *buf, size_t bufSize, size_t offset, const HarmonicsBlock &h) {
  int order = 3;
  for (int i = 0; i < HARMONIC_COUNT; i++) {
    offset += snprintf(buf + offset, bufSize - offset,
      "\"h%d_l1\":%.1f,\"h%d_l2\":%.1f,\"h%d_l3\":%.1f%s",
      order, h.h[i][0], order, h.h[i][1], order, h.h[i][2],
      (i < HARMONIC_COUNT - 1) ? "," : "");
    order += 2;
  }
  return offset;
}

void buildHarmonicsJson(char *buf, size_t bufSize, const HarmonicsBlock &h) {
  size_t offset = snprintf(buf, bufSize, "{\"thd1\":%.1f,\"thd2\":%.1f,\"thd3\":%.1f,", h.thd1, h.thd2, h.thd3);
  offset = appendHarmonics(buf, bufSize, offset, h);
  snprintf(buf + offset, bufSize - offset, "}");
}

// Basit JSON string alan cikarici -- ArduinoJson bagimliligi eklemeden
// {"ssid":"...","password":"..."} gibi duz bir govdeyi ayristirir.
String extractJsonString(const String &body, const String &key) {
  String pattern = "\"" + key + "\"";
  int idx = body.indexOf(pattern);
  if (idx < 0) return "";
  idx = body.indexOf(':', idx);
  if (idx < 0) return "";
  idx = body.indexOf('"', idx);
  if (idx < 0) return "";
  int end = body.indexOf('"', idx + 1);
  if (end < 0) return "";
  return body.substring(idx + 1, end);
}

void handleProvisionStatus() {
  provisionServer.send(200, "application/json", "{\"device_id\":\"" + deviceId + "\"}");
}

void handleProvisionConfigure() {
  String body = provisionServer.arg("plain");
  String ssid = extractJsonString(body, "ssid");
  String pass = extractJsonString(body, "password");
  if (ssid.length() == 0) {
    provisionServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"ssid gerekli\"}");
    return;
  }
  pendingSsid = ssid;
  pendingPass = pass;
  provisionServer.send(200, "application/json", "{\"status\":\"ok\"}");
  wifiConfigured = true;
}

// Ev WiFi agina baglanmayi dener, timeoutMs icinde sonuc alinamazsa false doner.
// TLS sertifika dogrulamasi cihazin saatine bagli: sertifikanin "gecerlilik
// baslangici" kontrol edilebilmeli. ESP32 epoch 0'dan (1970) basladigi icin,
// NTP olmadan her dogrulama "sertifika henuz gecerli degil" diye duser ve cihaz
// broker'a HIC baglanamaz. Bu yuzden TLS'ten once zaman senkronlanir.
bool senkronlaZaman() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Saat senkronlaniyor");
  unsigned long basla = millis();
  time_t simdi = time(nullptr);
  // 2020'den buyuk bir zaman gorene kadar bekle (epoch 0'dan uzaklasti mi)
  while (simdi < 1600000000 && millis() - basla < 15000) {
    delay(300);
    Serial.print(".");
    simdi = time(nullptr);
  }
  Serial.println();
  if (simdi < 1600000000) {
    Serial.println("Saat senkronlanamadi -- sertifika dogrulamasi basarisiz olacak.");
    return false;
  }
  Serial.print("Saat: ");
  Serial.print(ctime(&simdi));
  return true;
}


bool connectToWifi(const String &ssid, const String &pass, unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

// Cihazin kendi Access Point'ini acar ("BinaryEnerji-<cihaz id>", sifresiz) ve
// telefon uygulamasindan gelecek POST /configure ile ev WiFi bilgisini bekler.
// Donus degeri:
//   true  -> kayitli aga kendiliginden yeniden baglanildi (kullanici mudahalesi yok)
//   false -> kullanici telefondan yeni bilgi girdi; pendingSsid/pendingPass dolu
bool startProvisioningAP(const String &savedSsid, const String &savedPass) {
  // AP_STA: AP yayinda kalirken ayni anda STA ile kayitli agi da deneyebilelim.
  // Yalnizca WIFI_AP olsaydi telefon baglanabilirdi ama cihaz kendi agini hic
  // deneyemezdi -- kurtarmanin tamami buna bagli.
  WiFi.mode(WIFI_AP_STA);
  String apName = "BinaryEnerji-" + deviceId;
  WiFi.softAP(apName.c_str());
  Serial.print("Kurulum AP'si acildi: ");
  Serial.println(apName);
  Serial.println("Telefon uygulamasindan bu aga baglanip WiFi bilgilerini girin.");
  if (savedSsid.length() > 0) {
    Serial.print("Bu sirada kayitli ag da denenmeye devam edecek: ");
    Serial.println(savedSsid);
  }

  // Handler'lar yalnizca bir kez kaydedilmeli; bu fonksiyon kurulum basarisiz
  // olunca tekrar cagriliyor ve WebServer::on her cagrida listeye ekliyor.
  static bool handlerlarKuruldu = false;
  if (!handlerlarKuruldu) {
    provisionServer.on("/status", HTTP_GET, handleProvisionStatus);
    provisionServer.on("/configure", HTTP_POST, handleProvisionConfigure);
    handlerlarKuruldu = true;
  }
  provisionServer.begin();

  wifiConfigured = false;
  unsigned long sonDeneme = millis();
  unsigned long denemeBasi = 0;
  bool denemeSuruyor = false;

  while (!wifiConfigured) {
    // Telefon her an baglanabilmeli -- yeniden deneme bunu bloke etmemeli.
    provisionServer.handleClient();
    delay(2);

    if (savedSsid.length() == 0) continue;

    if (!denemeSuruyor && millis() - sonDeneme >= PROVISION_RETRY_INTERVAL_MS) {
      Serial.println("Kayitli ag yeniden deneniyor...");
      WiFi.begin(savedSsid.c_str(), savedPass.c_str());
      denemeSuruyor = true;
      denemeBasi = millis();
    }

    if (denemeSuruyor) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Kayitli aga yeniden baglanildi, kurulum modundan cikiliyor.");
        provisionServer.stop();
        return true;
      }
      if (millis() - denemeBasi >= PROVISION_RETRY_WINDOW_MS) {
        WiFi.disconnect();   // AP_STA'da yalnizca STA tarafini birakir
        denemeSuruyor = false;
        sonDeneme = millis();
      }
    }
  }
  provisionServer.stop();
  return false;
}

void setup() {
  Serial.begin(115200);
  // Excel haritasindaki "Modbus Hızı" varsayilani 19200 -- cihazin Ayarlar > Modbus
  // Ayarlari ekranindaki gercek degerle eslesmezse burasi guncellenmeli.
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);

  pinMode(RE_DE_PIN, OUTPUT);
  node.begin(1, Serial2); // TODO: cihazin Ayarlar > Modbus Ayarlari'ndaki gercek Modbus adresiyle karsilastir
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  setupDeviceId();
  Serial.print("CIHAZ ID (etikete/kutuya yazin): ");
  Serial.println(deviceId);

  Serial.print("Firmware surumu: ");
  Serial.println(FW_VERSION);

  otaPrefs.begin("ota", false);
  if (otaPrefs.getBool("pending", false)) {
    Serial.println("Bu, onaylanmamis bir OTA sonrasi ilk acilis -- onaylanamazsa geri donulecek.");
    otaPendingConfirm = true;
    otaPendingSince = millis();
  }

  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
  prefs.begin("wifi", false);

  if (digitalRead(WIFI_RESET_PIN) == LOW) {
    Serial.println("BOOT tusu basili -- kayitli WiFi bilgisi siliniyor, kurulum moduna giriliyor.");
    prefs.clear();
  }

  String savedSsid = prefs.getString("ssid", "");
  String savedPass = prefs.getString("pass", "");

  bool connected = false;
  if (savedSsid.length() > 0) {
    Serial.print("Kayitli aga baglaniliyor: ");
    Serial.println(savedSsid);
    connected = connectToWifi(savedSsid, savedPass, 15000);
  }

  while (!connected) {
    // Kurulum modu artik bir cikmaz sokak degil: kayitli ag geri gelirse
    // cihaz kendi kendine kurtulur.
    if (startProvisioningAP(savedSsid, savedPass)) {
      connected = true;
      break;
    }
    connected = connectToWifi(pendingSsid, pendingPass, 20000);
    if (connected) {
      prefs.putString("ssid", pendingSsid);
      prefs.putString("pass", pendingPass);
      savedSsid = pendingSsid;
      savedPass = pendingPass;
    } else {
      Serial.println("Girilen WiFi bilgileriyle baglanilamadi, kurulum moduna donuluyor.");
    }
  }

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);   // AP_STA'dan tamamen cik
  Serial.print("WiFi'ya baglanildi: ");
  Serial.println(WiFi.SSID());

  // Sertifika dogrulamasi cihazin SAATI DOGRUYSA calisir; ESP32 epoch 0'dan
  // basladigi icin NTP olmadan her dogrulama "sertifika henuz gecerli degil"
  // diye basarisiz olur. Bu yuzden zaman once senkronlaniyor.
  senkronlaZaman();
  espClient.setCACert(ISRG_ROOTS);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(onMqttMessage);
  // Harmonik/tepe-deger payload'lari buyuk (50+ alan) -- ANL13'ten daha genis
  // bir tampon gerekiyor, yoksa publish() sessizce basarisiz olur.
  mqttClient.setBufferSize(2048);

  // Cihaz kimlik bilgisi (seri no, urun/kart ID, versiyonlar) statik -- sadece
  // aciliste bir kez okunup MQTT'ye bir kez yayinlanir (retained).
  delay(30);
  deviceInfoOk = readDeviceInfo();
  if (!deviceInfoOk) Serial.println("Cihaz bilgisi okunamadi!");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiDisconnectedSince == 0) {
      wifiDisconnectedSince = millis();
      Serial.println("WiFi baglantisi koptu, yeniden baglanmaya calisiliyor...");
    }
    WiFi.reconnect();
    delay(2000);
    if (millis() - wifiDisconnectedSince > WIFI_RECONNECT_RESTART_MS) {
      Serial.println("WiFi uzun sure baglanamadi, cihaz yeniden baslatiliyor...");
      ESP.restart();
    }
    return;
  }
  wifiDisconnectedSince = 0;

  if (!mqttClient.connected()) {
    bool ok = mqttClient.connect(
      deviceId.c_str(), "esp32user", "***REDACTED-MQTT-PASSWORD***",
      statusTopic.c_str(), 1, true, "offline");
    if (ok) {
      mqttClient.subscribe(cmdTopic.c_str());
      mqttClient.subscribe(otaTopic.c_str());
      mqttClient.publish(statusTopic.c_str(), "online", true);
      tlsBasarisizSayaci = 0;
      // WiFi degisiminin sonucu ancak baglanti geri geldiginde bildirilebilir.
      if (wifiDegisimSonucu) {
        mqttClient.publish(
          wifiTopic.c_str(),
          wifiDegisimSonucu == 1
            ? "{\"ok\":true}"
            : "{\"ok\":false,\"hata\":\"yeni aga baglanilamadi, eski ag korundu\"}",
          true);
        wifiDegisimSonucu = 0;
      }
    } else {
      tlsBasarisizSayaci++;
      // EMNIYET YEDEGI: sertifika dogrulamasi surekli basarisiz olursa (kok
      // sertifika degismis, saat senkronlanamamis, ara sertifika zinciri
      // farklilasmis) cihaz TAMAMEN erisilemez hale gelirdi -- sahada yeniden
      // flash icin yerinde mudahale demek. Dogrulanmamis ama SIFRELI baglanti,
      // hic baglanti olmamasindan yegdir. Duruma gunlukte dikkat cekiliyor.
      if (tlsBasarisizSayaci >= 5 && !tlsDogrulamaKapali) {
        Serial.println("UYARI: TLS dogrulamasi 5 kez basarisiz -- dogrulamasiz TLS'e dusuluyor.");
        Serial.println("       (Saat senkronlanmadiysa sebep budur; 10 dakikada bir tekrar denenecek.)");
        espClient.setInsecure();
        tlsDogrulamaKapali = true;
        tlsTekrarDenemeMs = millis();
      }
    }
  }
  mqttClient.loop();

  // Dogrulamasiz moddan CIKMAYI dene. Dusus gecici olmali: saat gec
  // senkronlandiysa ya da aksaklik gectiyse cihaz kendiliginden dogrulanmis
  // TLS'e donsun. Aksi halde tek bir kotu an, cihazi bir daha flash'lanana
  // kadar zayif modda birakirdi.
  if (tlsDogrulamaKapali && millis() - tlsTekrarDenemeMs > TLS_DOGRULAMA_TEKRAR_MS) {
    tlsTekrarDenemeMs = millis();
    if (senkronlaZaman()) {
      Serial.println("Saat artik gecerli -- dogrulanmis TLS tekrar deneniyor.");
      espClient.setCACert(ISRG_ROOTS);
      tlsDogrulamaKapali = false;
      tlsBasarisizSayaci = 0;
      mqttClient.disconnect();   // yeni el sikismasi dogrulamali olsun
    }
  }

  checkOtaRollbackTimeout();

  // Cihaz bilgisi statik oldugu icin bir kez (retained) yayinlanir.
  if (deviceInfoOk && !deviceInfoPublished && mqttClient.connected()) {
    char infoPayload[400];
    snprintf(infoPayload, sizeof(infoPayload),
      "{\"seri_no\":%ld,\"urun_id\":%ld,\"kart_id\":%ld,\"sistem_versiyon\":%ld,"
      "\"ulke_kodu\":%ld,\"firma_kodu\":%ld,\"besleme_tipi\":%ld,\"ekran_tipi\":%ld,"
      "\"keyboard_tipi\":%ld,\"kutu_tipi\":%ld,\"klemens_tipi\":%ld,\"connections\":%ld,\"storage\":%ld}",
      (long)deviceInfo.seriNo, (long)deviceInfo.urunId, (long)deviceInfo.kartId, (long)deviceInfo.sistemVersiyon,
      (long)deviceInfo.ulkeKodu, (long)deviceInfo.firmaKodu, (long)deviceInfo.beslemeTipi, (long)deviceInfo.ekranTipi,
      (long)deviceInfo.keyboardTipi, (long)deviceInfo.kutuTipi, (long)deviceInfo.klemensTipi,
      (long)deviceInfo.connections, (long)deviceInfo.storage);
    mqttClient.publish(infoTopic.c_str(), infoPayload, true);
    Serial.println(infoPayload);
    deviceInfoPublished = true;
  }

  bool okLive = readPowerAnalyzer();
  if (okLive) confirmOtaIfPending();

  if (okLive) {
    char payload[800];
    snprintf(payload, sizeof(payload),
      "{\"v1\":%.1f,\"i1\":%.3f,\"p1\":%.0f,\"q1\":%.0f,\"s1\":%.0f,\"f1\":%.2f,\"cos1\":%.3f,\"pf1\":%.3f,\"thd1\":%.3f,\"thvd1\":%.3f,"
      "\"v2\":%.1f,\"i2\":%.3f,\"p2\":%.0f,\"q2\":%.0f,\"s2\":%.0f,\"f2\":%.2f,\"cos2\":%.3f,\"pf2\":%.3f,\"thd2\":%.3f,\"thvd2\":%.3f,"
      "\"v3\":%.1f,\"i3\":%.3f,\"p3\":%.0f,\"q3\":%.0f,\"s3\":%.0f,\"f3\":%.2f,\"cos3\":%.3f,\"pf3\":%.3f,\"thd3\":%.3f,\"thvd3\":%.3f,"
      "\"vN\":%.1f,\"iN\":%.3f,\"vL12\":%.1f,\"vL23\":%.1f,\"vL31\":%.1f}",
      data.v1, data.i1, data.p1, data.q1, data.s1, data.f1, data.cos1, data.pf1, data.thd1, data.thvd1,
      data.v2, data.i2, data.p2, data.q2, data.s2, data.f2, data.cos2, data.pf2, data.thd2, data.thvd2,
      data.v3, data.i3, data.p3, data.q3, data.s3, data.f3, data.cos3, data.pf3, data.thd3, data.thvd3,
      data.vN, data.iN, data.vL12, data.vL23, data.vL31);

    mqttClient.publish(liveTopic.c_str(), payload);
    Serial.println(payload);
  } else {
    Serial.println("Modbus okuma hatasi!");
  }

  // Enerji sayaclari yavas degistigi icin her dongude degil, ~30 saniyede bir okunur (2sn x 15)
  if (loopCount % 15 == 0) {
    delay(30); // RS-485 hattina ardisik istekler arasi toparlanma suresi
    bool energyOk = readEnergyBlock();
    delay(30);
    readCtRatio(); // basarisiz olursa ctRatio son bilinen degerinde kalir

    if (energyOk) {
      char energyPayload[350];
      snprintf(energyPayload, sizeof(energyPayload),
        "{\"active_wh_tuketim\":%lu,\"inductive_varh_tuketim\":%lu,\"capacitive_varh_tuketim\":%lu,"
        "\"active_wh_uretim\":%lu,\"inductive_varh_uretim\":%lu,\"capacitive_varh_uretim\":%lu,"
        "\"ct_ratio\":%u,\"fw_version\":\"%s\"}",
        energy.activeWhTuketim, energy.inductiveVArhTuketim, energy.capacitiveVArhTuketim,
        energy.activeWhUretim, energy.inductiveVArhUretim, energy.capacitiveVArhUretim,
        ctRatio, FW_VERSION);
      mqttClient.publish(energyTopic.c_str(), energyPayload);
      Serial.println(energyPayload);
    } else {
      Serial.println("Enerji okuma hatasi!");
    }

    delay(30);
    if (readTotals()) {
      char statsPayload[700];
      snprintf(statsPayload, sizeof(statsPayload),
        "{\"pActiveImp\":%.0f,\"pReactiveImp\":%.0f,\"pInductiveImp\":%.0f,\"pCapacitiveImp\":%.0f,\"pApparentImp\":%.0f,"
        "\"pActiveExp\":%.0f,\"pReactiveExp\":%.0f,\"pInductiveExp\":%.0f,\"pCapacitiveExp\":%.0f,\"pApparentExp\":%.0f,"
        "\"avgCurrentImp\":%.3f,\"avgActivePowerImp\":%.0f,\"avgCosImp\":%.3f,\"avgTanImp\":%.2f,\"avgPfImp\":%.3f,"
        "\"avgCurrentExp\":%.3f,\"avgActivePowerExp\":%.0f,\"avgCosExp\":%.3f,\"avgTanExp\":%.2f,\"avgPfExp\":%.3f,"
        "\"avgVoltageLN\":%.1f,\"avgVoltageLL\":%.1f,\"avgFrequency\":%.2f,\"avgThid\":%.1f,\"avgThvd\":%.1f}",
        totals.pActiveImp, totals.pReactiveImp, totals.pInductiveImp, totals.pCapacitiveImp, totals.pApparentImp,
        totals.pActiveExp, totals.pReactiveExp, totals.pInductiveExp, totals.pCapacitiveExp, totals.pApparentExp,
        totals.avgCurrentImp, totals.avgActivePowerImp, totals.avgCosImp, totals.avgTanImp, totals.avgPfImp,
        totals.avgCurrentExp, totals.avgActivePowerExp, totals.avgCosExp, totals.avgTanExp, totals.avgPfExp,
        totals.avgVoltageLN, totals.avgVoltageLL, totals.avgFrequency, totals.avgThid, totals.avgThvd);
      mqttClient.publish(statsTopic.c_str(), statsPayload);
      Serial.println(statsPayload);
    } else {
      Serial.println("Toplam/Ortalama okuma hatasi!");
    }
  }

  // Tepe degerleri ve demand daha da seyrek degistigi icin ~60 saniyede bir
  // okunur (2sn x 30). Her biri birkac parcali Modbus istegi gerektiriyor.
  // (Not: Kademeler/kompanzasyon rolesi blogu kaldirildi -- bu ANL21 birimi
  // kompanzasyon rolesi degil, o registerlar bu cihazda hic mevcut degil.)
  if (loopCount % 30 == 0) {
    delay(100);
    if (readPeakBlock(peaksTuketim, 11010)) {
      char buf[1400];
      buildPeakJson(buf, sizeof(buf), peaksTuketim);
      mqttClient.publish((peaksTopic + "/tuketim").c_str(), buf);
    } else {
      Serial.println("Tepe degeri (tuketim) okuma hatasi!");
    }

    delay(100);
    if (readPeakBlock(peaksUretim, 11260)) {
      char buf[1400];
      buildPeakJson(buf, sizeof(buf), peaksUretim);
      mqttClient.publish((peaksTopic + "/uretim").c_str(), buf);
    } else {
      Serial.println("Tepe degeri (uretim) okuma hatasi!");
    }

    delay(100);
    if (readDemandBlock(demandTuketim, 12010)) {
      char buf[1000];
      buildDemandJson(buf, sizeof(buf), demandTuketim);
      mqttClient.publish((demandTopic + "/tuketim").c_str(), buf);
    } else {
      Serial.println("Demand (tuketim) okuma hatasi!");
    }

    delay(100);
    if (readDemandBlock(demandUretim, 12260)) {
      char buf[1000];
      buildDemandJson(buf, sizeof(buf), demandUretim);
      mqttClient.publish((demandTopic + "/uretim").c_str(), buf);
    } else {
      Serial.println("Demand (uretim) okuma hatasi!");
    }
  }

  // Harmonikler en agir blok, gercek zamanli olmasina gerek yok -- ~5 dakikada
  // bir okunur (2sn x 150).
  if (loopCount % 150 == 0) {
    delay(30);
    if (readHarmonicsBlock(harmCurrent, 30000)) {
      char buf[1400];
      buildHarmonicsJson(buf, sizeof(buf), harmCurrent);
      mqttClient.publish((harmonicsTopic + "/akim").c_str(), buf);
    } else {
      Serial.println("Akim harmonikleri okuma hatasi!");
    }

    delay(30);
    if (readHarmonicsBlock(harmVoltage, 20000)) {
      char buf[1400];
      buildHarmonicsJson(buf, sizeof(buf), harmVoltage);
      mqttClient.publish((harmonicsTopic + "/gerilim").c_str(), buf);
    } else {
      Serial.println("Gerilim harmonikleri okuma hatasi!");
    }
  }

  loopCount++;
  delay(2000);
}
