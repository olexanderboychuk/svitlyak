#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <time.h>

// ================= ПІДКЛЮЧЕННЯ =================
#define PIN_RING D4
#define PIN_PIR  D5
#define PIN_VIB  D6     // вібромотор (через транзистор + діод!)
#define PIN_LDR  A0
#define NUM_LEDS 24

// ================= НАЛАШТУВАННЯ: ЗНАЧЕННЯ ЗА ЗАМОВЧУВАННЯМ =================
// Усе, що нижче, можна змінити з веб-інтерфейсу (зберігається в EEPROM).
#define DEF_MIN_BRIGHTNESS   15     // яскравість при світлі
#define DEF_MAX_BRIGHTNESS   100    // яскравість у темряві
#define DEF_DAY_LEVEL        450    // поріг "день": LVL вище -> день
#define DEF_NIGHT_ARM        390    // рух приймається лише якщо LVL нижче цього
#define DEF_HOLD_SEC         10     // світити стільки секунд після останнього руху
#define DEF_WAVE_MS          2500   // плавне затухання; 0 = гаснути одразу
#define DEF_VIB_MS           0      // тривалість вібрації; 0 = без вібрації (мотор трясе PIR)
#define DEF_WARMUP_SEC       60     // прогрів HC-SR501 (перші 60 с після старту — не рух)
#define DEF_WINDOW_MIN       42     // скільки з 50 семплів (500 мс) мають бути HIGH
#define DEF_PIR_ENABLED      1      // 1 = рух є джерелом вмикання; 0 = PIR мовчить
#define DEF_PIR_OFF_ONLY     0      // рух під час світіння продовжує таймер
#define DEF_PIR_COOL_SEC     25     // пауза PIR після вимкнення лампи (кільце охолоджується)
#define DEF_PIR_MAX_RUN_SEC   0     // 0 = серію будь-якої довжини рахуємо (режим «H» датчика)
#define PIR_MIN_RUN_MS        300UL  // серія коротша = EMI-тик, не рух
#define DEF_LDR_DARK_HIGH    1      // 1 = при затемненні A0 зростає (LDR між A0 і GND)
#define DEF_SCHED_ENABLED    0      // 0 = фільтр за графіком відключень вимкнено
#define DEF_SCHED_SOURCE     0      // 0 = Чернівціобленерго; інші = безкоштовні регіональні дзеркала
#define DEF_SCHED_GROUP      1      // черга (група) 1..12
#define DEF_SCHED_SOFT       0      // 0 = «МЗ» (можливо ЗАЖИВЛЕНІ) НЕ є відключенням; лише «В»
#define DEF_SCHED_FAILOPEN   1      // 1 = якщо графік недоступний, лампа працює як звичайно
#define DEF_ALERT_ENABLED    0      // 0 = тривоги NEPTUN вимкнено
#define DEF_ALERT_RAION      1      // № району в списку (1..138)
#define DEF_ALERT_POLL       60     // перевіряти кожних 60 с
#define DEF_ALERT_CLEAR      1      // 1 = сповіщати «відбій» зеленим

// ================= ФІКСОВАНІ ПАРАМЕТРИ =================
#define LDR_READ_MS      100UL  // як часто читати LDR
#define BRIGHT_STEP_MS   40UL   // як швидко яскравість рухається до цілі
#define BRIGHT_DEADBAND  2      // різниця, нижче якої яскравість не змінюється
#define DAYLIGHT_HYST    60     // гістерезис дня/ночі
#define DAYLIGHT_SETTLE_MS 2000UL
#define SAMPLE_MS        10UL
#define PIR_LOW_STABLE_MS 500UL  // повторне озброєння лише після стабільного LOW
#define WINDOW_SAMPLES   50
#define DEF_PIR_STUCK_SEC 60      // HIGH довше = залипання; 0 = захист вимкнено
#define WAVE_FRAME       20UL
#define WAVE_SPEED       40

// ================= WI-FI / AP =================
#define AP_PREFIX    "Svitlyak-"     // точка доступу: Svitlyak-1a2b3c
#define AP_PASSWORD  "svitlyak123"   // мінімум 8 символів; "" = відкрита мережа
#define MDNS_PREFIX  "svitlyak"      // mDNS: http://svitlyak-1a2b.local/
#define MDNS_HOSTNAME "nightlight"   // (не використовується: ім'я будується з ID чіпа)
#define WIFI_TIMEOUT 20000UL         // скільки чекати підключення до роутера
#define SCAN_TTL_MS  15000UL         // кеш списку мереж
#define MAX_NETS     24
#define AP_LINGER_MS 8000UL          // показати домашній IP, потім закрити AP навіть з підключеним телефоном
#define AP_SCAN_TTL_MS 120000UL      // повторне відкриття iOS-порталу не запускає новий радіоскан
#define AP_RETRY_MS  15000UL          // AP чекаємо лише коли кредлів немає (перше налаштування)
// HC-SR501 стоїть поруч з антеною: 20.5 dBm (дефолт SDK) дає наводку на D5.
// 8.5 dBm різко зменшує 2.4-ГГц поле й імпульси струму, дальності Wi-Fi вистачає.
#define WIFI_TX_DBM  8.5f

// ================= ГРАФІК ВІДКЛЮЧЕНЬ =================
// Джерело: https://oblenergo.cv.ua/shutdowns/ (HTML-графік, 48 слотів по 30 хв).
#define SCHED_HOST        "oblenergo.cv.ua"
#define SCHED_PATH        "/shutdowns/"
#define SCHED_SLOTS       48          // 30-хвилинних слотів на добу
#define SCHED_GROUPS      12          // черг на сторінці
#define SCHED_REFRESH_MS  900000UL    // успішне оновлення -> раз на 15 хв
#define SCHED_RETRY_MS    120000UL    // помилка -> повтор через 2 хв
#define SCHED_TIMEOUT_MS  25000UL     // загальний таймаут запиту
#define SCHED_MAX_BYTES   60000UL     // запобіжник від нескінченного читання
#define TZ_INFO "EET-2EEST,M3.5.0/3,M10.5.0/4"   // Europe/Kyiv

// Безкоштовні open-data дзеркала. Формат однаковий: fact.data.<date>.GPVx.<hour>.
#define SCHED_JSON_HOST   "raw.githubusercontent.com"
#define SCHED_REGION_COUNT 21
#define SCHED_SOURCE_MYKOLAIV 18
#define SCHED_SOURCE_KIROVOHRAD 19
#define SCHED_SOURCE_SUMY 20
const char SCHED_REGION_DATA[] PROGMEM =
"Чернівецька (офіційний сайт)\t\n"
"м. Київ\t/Baskerville42/outage-data-ua/main/data/kyiv.json\n"
"Київська\t/Baskerville42/outage-data-ua/main/data/kyiv-region.json\n"
"Дніпропетровська\t/Baskerville42/outage-data-ua/main/data/dnipro.json\n"
"Одеська\t/Baskerville42/outage-data-ua/main/data/odesa.json\n"
"Вінницька\t/vn-progr/gpv-voe-vinnytsia/main/data/Vinnytsiaoblenerho.json\n"
"Черкаська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Cherkasyoblenergo.json\n"
"Чернігівська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Chernihivoblenergo.json\n"
"Харківська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Kharkivoblenerho.json\n"
"Хмельницька\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Khmelnytskoblenerho.json\n"
"Львівська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Lvivoblenerho.json\n"
"Полтавська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Poltavaoblenergo.json\n"
"Івано-Франківська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Prykarpattiaoblenerho.json\n"
"Рівненська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Rivneoblenergo.json\n"
"Тернопільська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Ternopiloblenerho.json\n"
"Закарпатська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Zakarpattiaoblenerho.json\n"
"Запорізька\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Zaporizhzhiaoblenergo.json\n"
"Житомирська\t/yaroslav2901/OE_OUTAGE_DATA/main/data/Zhytomyroblenergo.json\n"
"Миколаївська (офіційний API)\t/api/v2/schedule/active\n"
"Кіровоградська (офіційний Telegram)\t\n"
"Сумська (офіційний сайт)\t\n";

// ================= ПОВІТРЯНІ ТРИВОГИ (NEPTUN) =================
// Джерело: https://neptun.in.ua/api/v1/alerts — JSON лише активних районів (~7 KB).
#define ALERT_HOST        "neptun.in.ua"
#define ALERT_PATH        "/api/v1/alerts"
#define ALERT_UA          "nightlight/1.0 (ESP8266)"   // браузерний UA Cloudflare блокує (403)
#define RAION_COUNT       138
#define ALERT_DEF_POLL    60          // перевіряти кожних 60 с
#define ALERT_RETRY_MS    20000UL     // помилка -> повтор через 20 с
#define ALERT_TIMEOUT_MS  20000UL     // таймаут запиту
#define ALERT_NOTIFY_MS   5000UL      // сповіщення триває 5 с
#define ALERT_BLINK_MS    250UL       // період блима
#define ALERT_VIBE_ON     120UL       // вібрація: 120 увімк / 80 вимк
#define ALERT_VIBE_CYCLE  200UL

// Список районів: "key\tобласть - район\n" (138 записів, ~12 KB флешу)
const char RAION_DATA[] PROGMEM =
"бахчисарайський\tавтономна республіка крим — Бахчисарайський р-н\n"
"білогірський\tавтономна республіка крим — Білогірський р-н\n"
"джанкойський\tавтономна республіка крим — Джанкойський р-н\n"
"євпаторійський\tавтономна республіка крим — Євпаторійський р-н\n"
"керченський\tавтономна республіка крим — Керченський р-н\n"
"курманський\tавтономна республіка крим — Курманський р-н\n"
"перекопський\tавтономна республіка крим — Перекопський р-н\n"
"сімферопольський\tавтономна республіка крим — Сімферопольський р-н\n"
"феодосійський\tавтономна республіка крим — Феодосійський р-н\n"
"ялтинський\tавтономна республіка крим — Ялтинський р-н\n"
"вінницький\tвінницька — Вінницький р-н\n"
"гайсинський\tвінницька — Гайсинський р-н\n"
"жмеринський\tвінницька — Жмеринський р-н\n"
"могилів-подільський\tвінницька — Могилів-Подільський р-н\n"
"тульчинський\tвінницька — Тульчинський р-н\n"
"хмільницький\tвінницька — Хмільницький р-н\n"
"володимир-волинський\tволинська — Володимир-Волинський р-н\n"
"камінь-каширський\tволинська — Камінь-Каширський р-н\n"
"ковельський\tволинська — Ковельський р-н\n"
"луцький\tволинська — Луцький р-н\n"
"дніпровський\tдніпропетровська — Дніпровський р-н\n"
"кам'янський\tдніпропетровська — Кам’янський р-н\n"
"криворізький\tдніпропетровська — Криворізький р-н\n"
"нікопольський\tдніпропетровська — Нікопольський р-н\n"
"павлоградський\tдніпропетровська — Павлоградський р-н\n"
"новомосковський\tдніпропетровська — Самарівський р-н\n"
"синельниківський\tдніпропетровська — Синельниківський р-н\n"
"бахмутський\tдонецька — Бахмутський р-н\n"
"волноваський\tдонецька — Волноваський р-н\n"
"горлівський\tдонецька — Горлівський р-н\n"
"донецький\tдонецька — Донецький р-н\n"
"кальміуський\tдонецька — Кальміуський р-н\n"
"краматорський\tдонецька — Краматорський р-н\n"
"маріупольський\tдонецька — Маріупольський р-н\n"
"покровський\tдонецька — Покровський р-н\n"
"бердичівський\tжитомирська — Бердичівський р-н\n"
"житомирський\tжитомирська — Житомирський р-н\n"
"новоград-волинський\tжитомирська — Звягельський р-н\n"
"коростенський\tжитомирська — Коростенський р-н\n"
"берегівський\tзакарпатська — Берегівський р-н\n"
"мукачівський\tзакарпатська — Мукачівський р-н\n"
"рахівський\tзакарпатська — Рахівський р-н\n"
"тячівський\tзакарпатська — Тячівський р-н\n"
"ужгородський\tзакарпатська — Ужгородський р-н\n"
"хустський\tзакарпатська — Хустський р-н\n"
"бердянський\tзапорізька — Бердянський р-н\n"
"василівський\tзапорізька — Василівський р-н\n"
"запорізький\tзапорізька — Запорізький р-н\n"
"мелітопольський\tзапорізька — Мелітопольський р-н\n"
"пологівський\tзапорізька — Пологівський р-н\n"
"верховинський\tівано-франківська — Верховинський р-н\n"
"івано-франківський\tівано-франківська — Івано-Франківський р-н\n"
"калуський\tівано-франківська — Калуський р-н\n"
"коломийський\tівано-франківська — Коломийський р-н\n"
"косівський\tівано-франківська — Косівський р-н\n"
"надвірнянський\tівано-франківська — Надвірнянський р-н\n"
"білоцерківський\tкиївська — Білоцерківський р-н\n"
"бориспільський\tкиївська — Бориспільський р-н\n"
"броварський\tкиївська — Броварський р-н\n"
"бучанський\tкиївська — Бучанський р-н\n"
"вишгородський\tкиївська — Вишгородський р-н\n"
"обухівський\tкиївська — Обухівський р-н\n"
"фастівський\tкиївська — Фастівський р-н\n"
"голованівський\tкіровоградська — Голованівський р-н\n"
"кропивницький\tкіровоградська — Кропивницький р-н\n"
"новоукраїнський\tкіровоградська — Новоукраїнський р-н\n"
"олександрійський\tкіровоградська — Олександрійський р-н\n"
"алчевський\tлуганська — Алчевський р-н\n"
"довжанський\tлуганська — Довжанський р-н\n"
"луганський\tлуганська — Луганський р-н\n"
"ровеньківський\tлуганська — Ровеньківський р-н\n"
"сватівський\tлуганська — Сватівський р-н\n"
"сєвєродонецький\tлуганська — Сєвєродонецький р-н\n"
"старобільський\tлуганська — Старобільський р-н\n"
"щастинський\tлуганська — Щастинський р-н\n"
"дрогобицький\tльвівська — Дрогобицький р-н\n"
"золочівський\tльвівська — Золочівський р-н\n"
"львівський\tльвівська — Львівський р-н\n"
"самбірський\tльвівська — Самбірський р-н\n"
"стрийський\tльвівська — Стрийський р-н\n"
"червоноградський\tльвівська — Червоноградський р-н\n"
"яворівський\tльвівська — Яворівський р-н\n"
"м. київ\tм. київ — м. Київ\n"
"баштанський\tмиколаївська — Баштанський р-н\n"
"вознесенський\tмиколаївська — Вознесенський р-н\n"
"миколаївський\tмиколаївська — Миколаївський р-н\n"
"первомайський\tмиколаївська — Первомайський р-н\n"
"березівський\tодеська — Березівський р-н\n"
"білгород-дністровський\tодеська — Білгород-Дністровський р-н\n"
"болградський\tодеська — Болградський р-н\n"
"ізмаїльський\tодеська — Ізмаїльський р-н\n"
"одеський\tодеська — Одеський р-н\n"
"подільський\tодеська — Подільський р-н\n"
"роздільнянський\tодеська — Роздільнянський р-н\n"
"кременчуцький\tполтавська — Кременчуцький р-н\n"
"лубенський\tполтавська — Лубенський р-н\n"
"миргородський\tполтавська — Миргородський р-н\n"
"полтавський\tполтавська — Полтавський р-н\n"
"вараський\tрівненська — Вараський р-н\n"
"дубенський\tрівненська — Дубенський р-н\n"
"рівненський\tрівненська — Рівненський р-н\n"
"сарненський\tрівненська — Сарненський р-н\n"
"севастополь\tсевастополь — м. Севастополь\n"
"конотопський\tсумська — Конотопський р-н\n"
"охтирський\tсумська — Охтирський р-н\n"
"роменський\tсумська — Роменський р-н\n"
"сумський\tсумська — Сумський р-н\n"
"шосткинський\tсумська — Шосткинський р-н\n"
"кременецький\tтернопільська — Кременецький р-н\n"
"тернопільський\tтернопільська — Тернопільський р-н\n"
"чортківський\tтернопільська — Чортківський р-н\n"
"богодухівський\tхарківська — Богодухівський р-н\n"
"ізюмський\tхарківська — Ізюмський р-н\n"
"красноградський\tхарківська — Красноградський р-н\n"
"куп'янський\tхарківська — Куп’янський р-н\n"
"лозівський\tхарківська — Лозівський р-н\n"
"харківський\tхарківська — Харківський р-н\n"
"чугуївський\tхарківська — Чугуївський р-н\n"
"бериславський\tхерсонська — Бериславський р-н\n"
"генічеський\tхерсонська — Генічеський р-н\n"
"каховський\tхерсонська — Каховський р-н\n"
"скадовський\tхерсонська — Скадовський р-н\n"
"херсонський\tхерсонська — Херсонський р-н\n"
"кам'янець-подільський\tхмельницька — Кам’янець-Подільський р-н\n"
"хмельницький\tхмельницька — Хмельницький р-н\n"
"шепетівський\tхмельницька — Шепетівський р-н\n"
"звенигородський\tчеркаська — Звенигородський р-н\n"
"золотоніський\tчеркаська — Золотоніський р-н\n"
"уманський\tчеркаська — Уманський р-н\n"
"черкаський\tчеркаська — Черкаський р-н\n"
"вижницький\tчернівецька — Вижницький р-н\n"
"дністровський\tчернівецька — Дністровський р-н\n"
"чернівецький\tчернівецька — Чернівецький р-н\n"
"корюківський\tчернігівська — Корюківський р-н\n"
"ніжинський\tчернігівська — Ніжинський р-н\n"
"новгород-сіверський\tчернігівська — Новгород-Сіверський р-н\n"
"прилуцький\tчернігівська — Прилуцький р-н\n"
"чернігівський\tчернігівська — Чернігівський р-н\n";



// forward: хелпери списку районів оголошені нижче (table у flash)
void raionKey(uint16_t idx, char* buf, uint8_t max);
void raionAppend(String& p, uint16_t idx);
void pageWizard();

// ================= СТАНИ ЛАМПИ =================
// OFF   - не світить (вдень або вночі без руху)
// ON    - є рух у темряві -> світить
// WAVE  - руху немає -> плавно гасне
// ALERT - сповіщення про тривогу/відбій (блимає, вібрує)
enum LampState { ST_OFF, ST_ON, ST_WAVE, ST_ALERT };

// ================= СТРУКТУРА НАЛАШТУВАНЬ =================
#define CFG_MAGIC 0x4E4C4351UL       // "NLCQ" v13 (профіль PIR: cool 25с, серія 0.3–10с)
#define CFG_MAGIC_V12 0x4E4C4350UL   // "NLCP" v12 — мігруємо без втрати мережі/графіка

struct Settings {
  uint32_t magic;
  uint16_t minBright;
  uint16_t maxBright;
  uint16_t dayLevel;
  uint16_t nightArm;
  uint16_t holdSec;
  uint16_t waveMs;
  uint16_t vibMs;
  uint16_t warmupSec;
  uint16_t windowMin;
  uint8_t  pirEnabled;        // 0 = датчик руху не є джерелом вмикання
  uint16_t pirStuckSec;       // HIGH довше за це = залипання (0 = вимк.)
  uint8_t  pirOffOnly;        // legacy: завжди 0, рух рахується у ST_ON і ST_WAVE
  uint16_t pirCoolSec;        // скільки чекати після вимкнення лампи (охолодження кільця)
  uint16_t pirMaxRunSec;      // серія HIGH довша = наводка/залипання (0 = вимк. фільтр)
  uint8_t  ldrDarkHigh;
  uint8_t  schedEnabled;
  uint8_t  schedGroup;
  uint8_t  schedSoft;
  uint8_t  schedFailOpen;
  uint8_t  alertEnabled;
  uint8_t  alertNotifyClear;
  uint8_t  alertLastLevel;    // запам'ятовуємо: чи вже блимали (переживає reboot)
  uint16_t alertRaion;        // 1..RAION_COUNT
  uint16_t alertPollSec;      // період перевірки
  char     alertKey[32];      // UTF-8 ключ района (щоб під час TLS не читати flash)
  uint8_t  setupDone;         // 1 = майстер першого запуску пройдено
  uint8_t  schedSource;       // індекс області у SCHED_REGION_DATA
  char     schedQueue[16];    // черга: "1.1" або "GPV1.1"
};

Settings cfg;

void settingsDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic       = CFG_MAGIC;
  cfg.minBright   = DEF_MIN_BRIGHTNESS;
  cfg.maxBright   = DEF_MAX_BRIGHTNESS;
  cfg.dayLevel    = DEF_DAY_LEVEL;
  cfg.nightArm    = DEF_NIGHT_ARM;
  cfg.holdSec     = DEF_HOLD_SEC;
  cfg.waveMs      = DEF_WAVE_MS;
  cfg.vibMs       = DEF_VIB_MS;
  cfg.warmupSec   = DEF_WARMUP_SEC;
  cfg.windowMin   = DEF_WINDOW_MIN;
  cfg.pirEnabled  = DEF_PIR_ENABLED;
  cfg.pirStuckSec = DEF_PIR_STUCK_SEC;
  cfg.pirOffOnly  = DEF_PIR_OFF_ONLY;
  cfg.pirCoolSec  = DEF_PIR_COOL_SEC;
  cfg.pirMaxRunSec = DEF_PIR_MAX_RUN_SEC;
  cfg.ldrDarkHigh = DEF_LDR_DARK_HIGH;
  cfg.schedEnabled  = DEF_SCHED_ENABLED;
  cfg.schedSource   = DEF_SCHED_SOURCE;
  strcpy(cfg.schedQueue, "1.1");
  cfg.schedGroup    = DEF_SCHED_GROUP;
  cfg.schedSoft     = DEF_SCHED_SOFT;
  cfg.schedFailOpen = DEF_SCHED_FAILOPEN;
  cfg.alertEnabled     = DEF_ALERT_ENABLED;
  cfg.alertRaion      = DEF_ALERT_RAION;
  cfg.alertPollSec    = DEF_ALERT_POLL;
  cfg.alertNotifyClear = DEF_ALERT_CLEAR;
  cfg.alertLastLevel  = 0;
  cfg.setupDone       = 0;         // новий пристрій -> показуємо майстер
}

void settingsSanitize() {
  if (cfg.minBright < 1)    cfg.minBright = 1;
  if (cfg.maxBright <= cfg.minBright) cfg.maxBright = cfg.minBright + 1;
  if (cfg.maxBright > 255)  cfg.maxBright = 255;
  if (cfg.dayLevel > 1023)  cfg.dayLevel = 1023;
  if (cfg.nightArm > cfg.dayLevel) cfg.nightArm = cfg.dayLevel;
  if (cfg.holdSec < 1)      cfg.holdSec = 1;
  if (cfg.holdSec > 600)    cfg.holdSec = 600;
  if (cfg.waveMs > 10000)   cfg.waveMs = 10000;
  if (cfg.vibMs > 2000)     cfg.vibMs = 2000;
  if (cfg.warmupSec > 600)  cfg.warmupSec = 600;
  if (cfg.windowMin < 1)    cfg.windowMin = 1;
  if (cfg.windowMin > WINDOW_SAMPLES) cfg.windowMin = WINDOW_SAMPLES;
  if (cfg.ldrDarkHigh > 1)  cfg.ldrDarkHigh = 1;
  if (cfg.pirEnabled > 1)   cfg.pirEnabled = 1;
  if (cfg.pirStuckSec > 600) cfg.pirStuckSec = 600;
  cfg.pirOffOnly = 0;        // безперервний рух завжди поновлює таймер лампи
  if (cfg.pirCoolSec > 300)  cfg.pirCoolSec = 300;
  if (cfg.pirMaxRunSec > 60) cfg.pirMaxRunSec = 60;
  if (cfg.schedEnabled > 1) cfg.schedEnabled = 0;
  // Старі EEPROM-записи закінчувались на setupDone; 0xFF у новому хвості
  // означає міграцію в сумісний режим Чернівців.
  if (cfg.schedSource >= SCHED_REGION_COUNT) {
    cfg.schedSource = 0;
    strcpy(cfg.schedQueue, "1.1");
  }
  cfg.schedQueue[sizeof(cfg.schedQueue) - 1] = 0;
  if (!cfg.schedQueue[0]) strcpy(cfg.schedQueue, "1.1");
  if (cfg.schedGroup < 1)   cfg.schedGroup = 1;
  if (cfg.schedGroup > SCHED_GROUPS) cfg.schedGroup = SCHED_GROUPS;
  if (cfg.schedSoft > 1)    cfg.schedSoft = 1;
  if (cfg.schedFailOpen > 1) cfg.schedFailOpen = 1;
  if (cfg.alertEnabled > 1)     cfg.alertEnabled = 0;
  if (cfg.alertNotifyClear > 1) cfg.alertNotifyClear = 1;
  if (cfg.alertLastLevel > 2)   cfg.alertLastLevel = 0;
  if (cfg.alertRaion < 1)       cfg.alertRaion = 1;
  if (cfg.alertRaion > RAION_COUNT) cfg.alertRaion = RAION_COUNT;
  if (cfg.alertPollSec < 15)    cfg.alertPollSec = 15;
  if (cfg.alertPollSec > 600)   cfg.alertPollSec = 600;
  cfg.alertKey[31] = 0;          // гарантуємо завершеність ключа
  if (cfg.setupDone > 1)         cfg.setupDone = 0;
}

// ================= КОЛІР =================
#define WARM_R 255
#define WARM_G 120
#define WARM_B 0

Adafruit_NeoPixel ring(NUM_LEDS, PIN_RING, NEO_GRB + NEO_KHZ800);

LampState lampState = ST_OFF;
LampState shownState = ST_WAVE;   // гарантує перше перемалювання

bool window[WINDOW_SAMPLES];
int winIndex = 0;
int highCount = 0;

unsigned long lastSample = 0;
unsigned long lastPrint = 0;
unsigned long motionUntil = 0;
unsigned long lampOnAt = 0;      // коли лампа востаннє засвітилась (для адаптивної паузи)
// Діагностика «чому лампа змінила стан» — видно на /live, щоб не гадати.
const char* lampWhy = "boot";
uint32_t lampChanges = 0;
uint32_t bootCount = 0;
unsigned long warmupMs = (unsigned long)DEF_WARMUP_SEC * 1000UL;
String resetReason = "?";
unsigned long waveStart = 0;
unsigned long lastFrame = 0;
unsigned long bootAt = 0;

unsigned long lastLdr = 0;
unsigned long lastBright = 0;
unsigned long vibUntil = 0;
bool vibActive = false;

bool daylight = false;
bool dayCandidate = false;
unsigned long dayCandidateAt = 0;
bool ringLit = false;

bool motionNow = false;bool motionPulse = false;      // завершена КОРОТКА серія HIGH = справжній рух (ставиться в samplePir)
bool pirPinHigh = false;       // поточний рівень піна (для чесного motionNow)
const char* runVerdict = "";   // вердикт останньої серії для логу
bool rawHighTracked = false;
unsigned long rawHighSince = 0;
bool pirStuck = false;
// «сліпа зона» PIR: під час зміни яскравості кільце дає ІЧ/електричний стрибок,
// який HC-SR501 приймає за рух -> лампа гасне і одразу вмикається знову.
unsigned long pirBlankUntil = 0;
// Після старту/затухання PIR може тримати HIGH сам по собі (розігрів HC-SR501,
// просадка живлення, нагрів кільця). Тому після паузи чекаємо хоч один LOW —
// інакше лампа вмикалась би знову і знову.
bool pirWaitLow = true;
bool pirLowTracked = false;
unsigned long pirLowSince = 0;
bool pirRunBlocked = false;

// ---- наводка Wi-Fi на PIR ----
// 2.4-ГГц TX ESP ловить високоомний вхід HC-SR501 (він поруч з антеною).
// Кожен наш HTTPS-запит (тривоги/графік) дає пачку сплесків -> датчик видає
// «рух» на 3–8 с. Позначаємо вікно запиту, і серію HIGH, що почалася в ньому,
// вважаємо наводкою, а не рухом.
unsigned long radioMarkFrom = 0, radioMarkTo = 0;
uint16_t radioNoiseRuns = 0;
bool pirRunFalse = false;

// Засліпити PIR на ms. Використовується, коли кільце розпалюється/затухає
// (тепле повітря крізь лінзу) і коли трясе вібромотор. Після паузи датчик
// озброюється лише після стабільного LOW — інакше HIGH «залипав» би назавжди.
void pirBlindFor(unsigned long ms) {
  unsigned long until = millis() + ms;
  if ((long)(until - pirBlankUntil) > 0) pirBlankUntil = until;
  pirWaitLow = true;
  pirLowTracked = false;
}

void markRadioNoiseStart() {
  unsigned long now = millis();
  radioMarkFrom = now - 400UL;
  radioMarkTo   = now + 9000UL;          // запобіжник, якщо запит затягнеться
}
void markRadioNoiseStop() {
  unsigned long now = millis();
  unsigned long t = now + 900UL;         // хвіст: датчик ще 1–2 с «дотримує»
  if ((long)(t - radioMarkTo) < 0) radioMarkTo = t;
}

bool pirGated(unsigned long now) {
  (void)now;
  return pirWaitLow;
}

// діагностика PIR: порахувати й виміряти кожен сплеск HIGH, бо HC-SR501
// стрибає й без руху (розігрів, теплові протяги, вібрація, просадки живлення)
bool lastRawPir = false;
int  pirEvents = 0;                 // LOW->HIGH переходів від старту
unsigned long pirHighStart = 0;     // поточна HIGH почалась тут
unsigned long pirLastHighMs = 0;    // тривалість останньої завершеної HIGH
unsigned long pirHighNowMs = 0;     // тривалість поточної HIGH

// ---- журнал завершених серій PIR ----
// Дає відповідь «на що саме реагує датчик» без Serial: тривалість серії,
// щільність HIGH, чи була це наводка/сліпа зона, рівень LDR і лампа.
#define PIR_LOG_N 32
struct PirLogEntry {
  uint32_t startMs;
  uint16_t durMs;
  uint8_t  hc;       // HIGH у вікні 500 мс (0..50)
  uint8_t  code;     // 0 відкинуто, 1 рух, 2 наводка, 3 тик, 4 задовга, 5 рух(вікно)
  uint8_t  flags;    // 1 радіо, 2 blocked, 4 stuck, 8 день
  uint8_t  lvl;      // LDR/4
  uint8_t  br;
  uint8_t  lamp;
};
PirLogEntry pirLog[PIR_LOG_N];
uint8_t  pirLogHead = 0;
uint8_t  pirLogCount = 0;
uint16_t pirAccepted = 0;           // серій, схожих на рух
uint16_t pirRejected = 0;           // серій, відкинутих фільтрами

// ---------- графік відключень (компактно: 6 байт на добу) ----------
uint8_t  schedBits[6];          // 48 бітів: 1 = відключення (лампа може світити)
bool     schedValid = false;
uint8_t  schedDay = 0, schedMon = 0;
uint16_t schedYear = 0;
uint32_t schedSyncEpoch = 0;    // Unix-час останнього успішного оновлення
uint32_t schedNextAt = 0;       // millis() наступної спроби
uint8_t  schedErr = 0;          // код останньої помилки (0 = ок)
bool     schedForce = false;
bool     schedAllowedNow = true;   // кеш: чи можна зараз світити
bool     schedSlotOut = false;     // поточний слот = відключення
uint8_t  schedSlotNow = 0;

// ---------- повітряні тривоги ----------
bool     alertForce = false;
bool     alertValid = false;      // останній запит вдався
uint8_t  alertLevelNow = 0;       // рівень дляourїх района (0/1/2)
uint8_t  alertErr = 0;
uint32_t alertNextAt = 0;         // millis наступної перевірки
uint32_t alertCheckAt = 0;        // millis останньої перевірки
uint8_t  alertNotify = 0;         // 1=жовтий, 2=червоний, 3=зелений (відбій)
bool     alertBlinkOn = false;    // поточна фама сповіщення (блима увімкнено)
unsigned long alertNotifyStartAt = 0;
char     alertSince[6] = "";      // "HH:MM" з поля since (UTC)

float ldrSmooth = 512;
int ldrRaw = 0;
uint8_t currentBrightness = DEF_MIN_BRIGHTNESS;
uint8_t shownBrightness = 0;
uint8_t targetBrightness = DEF_MIN_BRIGHTNESS;
uint8_t waveFromBr = DEF_MIN_BRIGHTNESS;

// ================= ЧАС =================
// true, коли момент t уже настав. Коректно працює й після
// переповнення millis() (~49 діб), на відміну від "now >= t".
bool timeReached(unsigned long now, unsigned long t) {
  return (long)(now - t) >= 0;
}

// ================= ВІБРАЦІЯ =================
void startVibe() {
  if (cfg.vibMs == 0) return;          // вібрація вимкнена в налаштуваннях
  digitalWrite(PIN_VIB, HIGH);
  vibUntil = millis() + cfg.vibMs;
  vibActive = true;
  // Мотор трясе корпус — HC-SR501 бачить це як рух. Тримаємо датчик сліпим,
  // поки трясе, і ще ~0.8 с після, щоб механічний «дзвін» не дав фейк.
  pirBlindFor((unsigned long)cfg.vibMs + 800UL);
}

void updateVibe() {
  unsigned long now = millis();

  // Сповіщення тривоги: імпульсна вібрація протягом усього сповіщення
  if (alertNotify) {
    unsigned long el = now - alertNotifyStartAt;
    if (el < ALERT_NOTIFY_MS) {
      digitalWrite(PIN_VIB, (el % ALERT_VIBE_CYCLE) < ALERT_VIBE_ON ? HIGH : LOW);
      return;
    }
    digitalWrite(PIN_VIB, LOW);
    vibActive = false;
  }

  if (vibActive && timeReached(now, vibUntil)) {
    digitalWrite(PIN_VIB, LOW);
    vibActive = false;
  }
}

// ================= LDR =================
int readLdr() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_LDR);
    delayMicroseconds(200);
  }
  return sum / 8;
}

// Освітленість: 0 = повна темрява, 1023 = яскраве світло
int lightLevelFor(float raw) {
  int lvl = cfg.ldrDarkHigh ? 1023 - (int)raw : (int)raw;
  return constrain(lvl, 0, 1023);
}

// Освітленість -> яскравість: світло = MIN, темнота = MAX
uint8_t brightnessForLdr(float raw) {
  return constrain(
    map(lightLevelFor(raw), 0, 1023, cfg.maxBright, cfg.minBright),
    cfg.minBright, cfg.maxBright
  );
}

// Рух враховуємо лише в реальній темряві, а не в сірій зоні "день/ніч"
bool isReallyDark() {
  return lightLevelFor(ldrSmooth) < cfg.nightArm;
}

// День/ніч з гістерезисом і затримкою
void updateDayNight(unsigned long now) {
  // LDR має бути закритий від власного світла кільця. Освітленість
  // перевіряється в усіх станах, зокрема коли лампа вже світить.
  bool wantDay = daylight
    ? (lightLevelFor(ldrSmooth) >= (int)cfg.dayLevel - DAYLIGHT_HYST)
    : (lightLevelFor(ldrSmooth) >= (int)cfg.dayLevel);

  if (wantDay == daylight) {
    dayCandidate = false;
    return;
  }

  if (!dayCandidate) {
    dayCandidate = true;
    dayCandidateAt = now;
    return;
  }

  if (now - dayCandidateAt >= DAYLIGHT_SETTLE_MS) {
    daylight = wantDay;
    dayCandidate = false;
  }
}

void updateBrightness() {
  unsigned long now = millis();

  if (now - lastLdr >= LDR_READ_MS) {
    lastLdr = now;
    ldrRaw = readLdr();
    ldrSmooth += (ldrRaw - ldrSmooth) / 8.0;

    targetBrightness = brightnessForLdr(ldrSmooth);
    updateDayNight(now);
  }

  // повільно рухаємось до цілі, з мертвою зоною
  if (now - lastBright >= BRIGHT_STEP_MS) {
    lastBright = now;
    int diff = (int)targetBrightness - (int)currentBrightness;
    if (abs(diff) >= BRIGHT_DEADBAND) {
      currentBrightness += (diff > 0) ? 1 : -1;
    }
  }
}

// ================= СВІТЛОДІОДИ =================
void fillRing(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  ring.setBrightness(brightness);
  uint32_t c = ring.Color(r, g, b);
  for (int i = 0; i < NUM_LEDS; i++) {
    ring.setPixelColor(i, c);
  }
  ring.show();
}

void resetWindow() {
  for (int i = 0; i < WINDOW_SAMPLES; i++) window[i] = false;
  highCount = 0;
  winIndex = 0;
}

float lerpf(float a, float b, float t) {
  return a + (b - a) * t;
}

// Хвиля: тепле світло з райдужним переливом, плавно гасне до нуля
void drawWave(unsigned long elapsed) {
  float p = (float)elapsed / (float)cfg.waveMs;
  if (p > 1.0) p = 1.0;

  float w = sinf(PI * p);   // 0 на початку й у кінці, 1 посередині

  for (int i = 0; i < NUM_LEDS; i++) {
    uint16_t hue = (uint16_t)(i * (65536UL / NUM_LEDS) + elapsed * WAVE_SPEED);
    uint32_t rc = ring.gamma32(ring.ColorHSV(hue, 255, 255));
    float rr = (rc >> 16) & 0xFF;
    float rg = (rc >> 8) & 0xFF;
    float rb = rc & 0xFF;

    float lw = w * (0.65 + 0.35 * sinf(TWO_PI * ((float)i / NUM_LEDS - p * 2.0)));

    ring.setPixelColor(i,
      (uint8_t)lerpf(WARM_R, rr, lw),
      (uint8_t)lerpf(WARM_G, rg, lw),
      (uint8_t)lerpf(WARM_B, rb, lw));
  }

  ring.setBrightness((uint8_t)lerpf(waveFromBr, 0, p));
  ring.show();
}

const char* stateName(LampState s) {
  switch (s) {
    case ST_OFF:   return "OFF";
    case ST_ON:    return "ON";
    case ST_WAVE:  return "WAVE";
  }
  return "?";
}

void updateLeds() {
  unsigned long now = millis();

  // Сповіщення тривоги/відбою: блимає кольором рівня, рух паузований
  if (lampState == ST_ALERT) {
    ringLit = true;
    unsigned long el = now - alertNotifyStartAt;
    bool on = ((el / ALERT_BLINK_MS) % 2) == 0;
    if (on != alertBlinkOn) {
      alertBlinkOn = on;
      if (on) {
        if (alertNotify == 1)      fillRing(255, 180, 0, cfg.maxBright);  // жовтий
        else if (alertNotify == 2) fillRing(255, 0, 0, cfg.maxBright);    // червоний
        else                       fillRing(0, 255, 0, cfg.maxBright);     // зелений (відбій)
      } else {
        ring.clear();
        ring.show();
      }
    }
    shownState = ST_ALERT;
    shownBrightness = 0;
    return;
  }

  // Погашено: день або ніч без руху
  if (lampState == ST_OFF) {
    if (ringLit) {
      ring.clear();
      ring.show();
      ringLit = false;
    }
    shownState = ST_WAVE;      // примусово перемалювати при наступному вмиканні
    shownBrightness = 0;
    return;
  }

  ringLit = true;

  if (lampState == ST_WAVE) {
    if (now - lastFrame < WAVE_FRAME) return;
    lastFrame = now;
    drawWave(now - waveStart);
    shownState = ST_WAVE;
    return;
  }

  // ST_ON
  bool changed = (lampState != shownState);
  if (!changed && currentBrightness == shownBrightness) return;
  fillRing(WARM_R, WARM_G, WARM_B, currentBrightness);
  shownBrightness = currentBrightness;
  shownState = lampState;
}

// Опитування PIR: вікно семплів, вартовий проти залипання, облік кожного сплеску.
// Логи переходів LOW->HIGH дозволяють побачити хибні спрацювання (без руху).
bool samplePir(unsigned long now) {
  bool raw = digitalRead(PIN_PIR) == HIGH;
  pirPinHigh = raw;
  motionPulse = false;
  bool inhibited = (long)(now - pirBlankUntil) < 0 ||
                   now - bootAt < warmupMs ||
                   alertNotify || !cfg.pirEnabled || daylight;
  if (inhibited) {
    pirWaitLow = true;
    pirLowTracked = false;
    if (raw) pirRunBlocked = true;
    resetWindow();
  } else if (!raw) {
    if (!pirLowTracked) {
      pirLowTracked = true;
      pirLowSince = now;
    }
    if (now - pirLowSince >= PIR_LOW_STABLE_MS) pirWaitLow = false;
  }
  if (raw) pirLowTracked = false;

  if (raw != lastRawPir) {
    lastRawPir = raw;
    if (raw) {
      pirRunBlocked = inhibited || pirWaitLow;
      pirEvents++;
      pirHighStart = now;
      pirHighNowMs = 0;
      // чи збігся початок серії з вікном нашого Wi-Fi-запиту?
      pirRunFalse = ((long)(now - radioMarkFrom) >= 0) && ((long)(radioMarkTo - now) >= 0);
      if (pirRunFalse) radioNoiseRuns++;
      // діагностика: чому саме цей фронт може не стати рухом
      {
        bool blk = (long)(now - pirBlankUntil) < 0;
        bool lit = (lampState == ST_ON);
        Serial.printf("MOT? blk=%d gate=%d hc=%d runF=%d st=%d dark=%d allow=%d sch=%d lit=%d\n",
                      blk ? 1 : 0, pirGated(now) ? 1 : 0, (int)highCount, pirRunFalse ? 1 : 0,
                      pirStuck ? 1 : 0, isReallyDark() ? 1 : 0,
                      (cfg.pirOffOnly && lit) ? 0 : 1, schedAllowedNow ? 1 : 0, lit ? 1 : 0);
      }
    } else {
      pirLastHighMs = now - pirHighStart;
      pirHighNowMs = 0;
      // Справжній рух = серія HIGH від PIR_MIN_RUN_MS до pirMaxRunSec І достатньо
      // щільна в ковзному вікні 500 мс. Так відсікаються і EMI-тики (10–200 мс),
      // і довгі серії (тепло кільця, наводка, залипання).
      bool cleanRun = !pirRunFalse && !pirRunBlocked && !inhibited && !pirWaitLow;
      unsigned long runLong = (unsigned long)cfg.pirMaxRunSec * 1000UL;
      uint8_t code = 0;
      if (cfg.pirMaxRunSec > 0 && cleanRun &&
          pirLastHighMs >= PIR_MIN_RUN_MS && pirLastHighMs <= runLong &&
          highCount >= cfg.windowMin) {
        motionPulse = true;
        code = 1;
        runVerdict = " РУХ";
      } else if (pirRunFalse) {
        code = 2; runVerdict = " НАВОДКА";
      } else if (pirLastHighMs < PIR_MIN_RUN_MS) {
        code = 3; runVerdict = " ТИК";
      } else if (cfg.pirMaxRunSec > 0 && pirLastHighMs > runLong && cleanRun) {
        code = 4; runVerdict = " ЗАДОВГА";
      } else if (cfg.pirMaxRunSec == 0 && cleanRun && highCount >= cfg.windowMin) {
        code = 5; runVerdict = " РУХ (вікно)";
      } else {
        runVerdict = " ВІДКИНУТО";
      }
      // журнал: останні PIR_LOG_N серій, найновіша перезаписує найстарішу
      {
        PirLogEntry &e = pirLog[pirLogHead];
        e.startMs = pirHighStart;
        e.durMs   = pirLastHighMs > 65535UL ? 65535 : (uint16_t)pirLastHighMs;
        e.hc      = highCount > 255 ? 255 : (uint8_t)highCount;
        e.code    = code;
        e.flags   = (pirRunFalse ? 1 : 0) | (pirRunBlocked ? 2 : 0) |
                    (pirStuck ? 4 : 0) | (daylight ? 8 : 0);
        e.lvl     = (uint8_t)(lightLevelFor(ldrSmooth) / 4);
        e.br      = currentBrightness;
        e.lamp    = (uint8_t)lampState;
        pirLogHead = (pirLogHead + 1) % PIR_LOG_N;
        if (pirLogCount < PIR_LOG_N) pirLogCount++;
        if (code == 1 || code == 5) pirAccepted++; else pirRejected++;
      }
    }
    long pl = (long)(pirBlankUntil - now);
    Serial.printf("PIR: %d @%lu (подія %u, остання HIGH %lu ms) lamp=%d lvl=%d br=%d пауза=%ld с%s\n",
                  raw ? 1 : 0, now, pirEvents, pirLastHighMs, (int)lampState,
                  (int)lightLevelFor(ldrSmooth), (int)currentBrightness,
                  pl > 0 ? pl / 1000 : 0L, runVerdict);
    if (!raw) { pirRunFalse = false; runVerdict = ""; }   // серія скінчилась
  }
  if (raw) pirHighNowMs = now - pirHighStart;

  if (window[winIndex]) highCount--;
  window[winIndex] = raw && !inhibited && !pirWaitLow && !pirRunBlocked && !pirRunFalse;
  if (window[winIndex]) highCount++;
  winIndex = (winIndex + 1) % WINDOW_SAMPLES;

  // вартовий проти залиплого датчика (пірStuckSec=0 -> захист вимкнено)
  if (raw) {
    if (cfg.pirStuckSec > 0) {
      if (!rawHighTracked) {
        rawHighTracked = true;
        rawHighSince = now;
      } else if (now - rawHighSince >= (unsigned long)cfg.pirStuckSec * 1000UL) {
        pirStuck = true;
      }
    }
  } else {
    rawHighTracked = false;
    pirStuck = false;
  }
  return raw;
}

// ================= РУХ =================

// Лампа щойно згасла -> кільце ще гаряче і жене тепле повітря крізь лінзу PIR.
// Саме ці «теплові хвилі» давали хибні спрацювання через 10–50 с після вимкнення.
// Пауза масштабується реальним нагрівом: час світіння × яскравість. Якщо лампа
// блимнула на секунду — пауза мінімальна, і рух одразу знову вмикає світло.
void lampOffCooldown(unsigned long now) {
  if (cfg.pirCoolSec == 0) return;
  unsigned long litFor = now - lampOnAt;
  if (litFor > 30000UL) litFor = 30000UL;
  uint16_t br = currentBrightness ? currentBrightness : 1;
  unsigned long ms = (unsigned long)cfg.pirCoolSec * 1000UL;
  ms = ms * litFor / 30000UL;      // коротке світіння майже не гріє
  ms = ms * br / 255UL;            // тепло пропорційне яскравості
  if (ms < 1500UL) ms = 1500UL;    // мінімум — погасити ІЧ-стрибок кільця
  pirBlindFor(ms);
}

void updateMotion() {
  unsigned long now = millis();
  // Імпульс живе лише одну ітерацію; діагностика вдень не накопичує рух на ніч.
  motionPulse = false;
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    samplePir(now);
  }

  // Сповіщення тривоги: звичайна логіка руху паузований
  if (alertNotify) {
    motionNow = false;
    if (lampState != ST_ALERT) { lampWhy = "alert"; lampChanges++; }
    lampState = ST_ALERT;
    return;
  }

  // День: лампа не світить і рух не обробляємо, але датчик опитуємо —
  // інакше діагностика (події, HIGH у вікні) застигає на старих значеннях.
  if (daylight) {
    motionNow = false;
    if (lampState != ST_OFF) {
      lampState = ST_OFF;
      lampWhy = "daylight"; lampChanges++;
      lampOffCooldown(now);
      motionUntil = 0;
      digitalWrite(PIN_VIB, LOW);
      vibActive = false;
      resetWindow();
      rawHighTracked = false;
    }
    return;
  }

  // PIR вимкнено: рух НЕ є джерелом вмикання. Тоді лампа світить постійно,
  // коли темно і графік дозволяє, і плавно гасне, коли графік дає «світло є».
  if (!cfg.pirEnabled) {
    motionNow = false;
    if (isReallyDark() && schedAllowedNow) {
      if (lampState != ST_ON) {
        lampState = ST_ON;
        lampWhy = "always"; lampChanges++;
        lampOnAt = now;
        startVibe();
      }
      motionUntil = 0xFFFFFFFFUL;       // утримання не минатиме -> не гасне
    } else {
      motionUntil = 0;
      if (lampState == ST_ON) {
        if (cfg.waveMs > 0) {
          lampState = ST_WAVE;
          lampWhy = "always-off"; lampChanges++;
          waveStart = now;
          waveFromBr = currentBrightness;
          lastFrame = 0;
        } else {
          lampState = ST_OFF;
          lampWhy = "always-off"; lampChanges++;
          lampOffCooldown(now);
        }
      } else if (lampState == ST_WAVE &&
                 now - waveStart >= (unsigned long)cfg.waveMs) {
        lampState = ST_OFF;               // хвиля кінчена -> вимкнено
        lampWhy = "wave-end"; lampChanges++;
      }
    }
    return;
  }

  if (motionUntil == 0xFFFFFFFFUL)      // лишилось від режиму без PIR -> перевизначити
    motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;

  // Під час сліпої зони опитування триває, але рух не приймаємо.
  bool blanked = (long)(now - pirBlankUntil) < 0;

  // Початкове ввімкнення фільтрується; утримання ввімкненої лампи — за HIGH.
  bool motionAllowed = true;

  bool warmup = ((now - bootAt) < warmupMs);

  bool lit = (lampState == ST_ON || lampState == ST_WAVE);
  if (lit) {
    // Фільтри визначають лише право ПЕРШОГО ввімкнення. Після нього
    // кожен HIGH утримує світло, зокрема довга серія, RF-позначка,
    // залипання, пауза вікна або режим завершеної серії.
    // Відлік holdSec починається від останнього спостереженого HIGH.
    motionNow = pirPinHigh && schedAllowedNow;
    if (motionNow) motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;
  } else if (cfg.pirMaxRunSec > 0) {
    // Рух = завершена коротка серія. Тоді лампа не реагує ні на залипання
    // (HIGH хвилинами), ні на наводку Wi-Fi (серія довша або позначена).
    motionNow = false;
    if (motionPulse) {
      motionPulse = false;
      if (!blanked && !warmup && motionAllowed && !pirGated(now) && isReallyDark() && schedAllowedNow) {
        motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;
        motionNow = true;
      }
    }
  } else {
    bool highConfirmed = pirPinHigh && !pirStuck &&
                         pirHighNowMs >= PIR_MIN_RUN_MS &&
                         (highCount >= cfg.windowMin);
    // Для першого ввімкнення діють усі фільтри.
    bool cleanStart = !blanked && !warmup && !pirRunFalse &&
                      !pirRunBlocked && !pirGated(now);
    motionNow = motionAllowed && highConfirmed && cleanStart &&
                isReallyDark() && schedAllowedNow;
    if (motionNow) motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;
  }

  // графік відключень заборонив світити -> плавно гасимо
  if (!schedAllowedNow && lampState != ST_OFF) {
    if (cfg.waveMs > 0) {
      if (lampState != ST_WAVE) {
        lampState = ST_WAVE;
        lampWhy = "schedule"; lampChanges++;
        waveStart = now;
        waveFromBr = currentBrightness;
        lastFrame = 0;
        pirBlindFor((unsigned long)cfg.waveMs + 1200UL);
      }
    } else {
      lampState = ST_OFF;
      lampWhy = "schedule"; lampChanges++;
      lampOffCooldown(now);
    }
  }

  switch (lampState) {
    case ST_OFF:
      if (motionNow) {
        lampState = ST_ON;
        lampWhy = "motion"; lampChanges++;
        motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;
        lampOnAt = now;
        startVibe();
      }
      break;

    case ST_ON:
      if (timeReached(now, motionUntil)) {   // час утримання минув -> гаснемо
        if (cfg.waveMs > 0) {
          lampState = ST_WAVE;
          lampWhy = "hold-expired"; lampChanges++;
          waveStart = now;
          waveFromBr = currentBrightness;
          lastFrame = 0;
          // Сліпої зони тут НЕМА: якщо людина ще рухається, рух одразу
          // поверне лампу в ST_ON (див. case ST_WAVE).
        } else {
          lampState = ST_OFF;                // без хвилі - просто вимикаємось
          lampWhy = "hold-expired"; lampChanges++;
          lampOffCooldown(now);
        }
      }
      break;

    case ST_WAVE:
      if (motionNow) {                       // рух під час затухання -> знову світить
        lampState = ST_ON;
        lampWhy = "motion-wave"; lampChanges++;
        motionUntil = now + (unsigned long)cfg.holdSec * 1000UL;
        lampOnAt = now;
      } else if (now - waveStart >= (unsigned long)cfg.waveMs) {
        lampState = ST_OFF;
        lampWhy = "wave-end"; lampChanges++;
        lampOffCooldown(now);
      }
      break;
  }
}

// ================= WI-FI: AP + ПОРТАЛ =================
struct WifiCreds {
  uint32_t magic;
  char ssid[33];
  char pass[64];
};

static const uint32_t CRED_MAGIC = 0x4E4C3031;   // "NL01"

WifiCreds creds;
bool haveCreds = false;

ESP8266WebServer server(80);
DNSServer dnsServer;

struct NetInfo {
  String ssid;
  int32_t rssi;
  bool enc;
};

NetInfo nets[MAX_NETS];
int netCount = 0;
bool netsValid = false;
unsigned long netsAt = 0;
bool apScanPending = false;

bool connectPending = false;
bool apJoinPending = false;
unsigned long apJoinApplyAt = 0;
unsigned long connectStartedAt = 0;
unsigned long staRetryAt = 0;
#define STA_RETRY_MS 10000UL
unsigned long staScanAt = 0;
// ДАГОСЛИТ: тест «радіо вимкнено» для PIR
unsigned long wifiOffApplyAt = 0, wifiOffUntil = 0;
String apSsidStr;
String mdnsHost;                 // svitlyak-1a2b (унікальне ім'я пристрою в мережі)
unsigned long lastApPrint = 0;

bool apRunning = false;
unsigned long apStopAt = 0;
bool staDownTracked = false;
unsigned long staDownSince = 0;
bool mdnsRunning = false;
uint8_t staChannel = 0;   // канал роутера: AP має бути на тому ж каналі (ESP8266 STA+AP)

// ---------- збереження в EEPROM ----------
#define EEPROM_CFG_ADDR ((int)sizeof(WifiCreds))
#define EEPROM_SIZE     ((int)(sizeof(WifiCreds) + sizeof(Settings)))

void settingsLoad() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_CFG_ADDR, cfg);
  if (cfg.magic == CFG_MAGIC_V12) {
    // Оновлення прошивки: мережа, графік, тривоги й яскравість лишаються,
    // але профіль PIR перезаписуємо новими дефолтами. Інакше старі
    // 0/35/30/0 з EEPROM перекрили б нові значення після перепрошивки.
    cfg.magic        = CFG_MAGIC;
    cfg.vibMs        = DEF_VIB_MS;
    cfg.warmupSec    = DEF_WARMUP_SEC;
    cfg.windowMin    = DEF_WINDOW_MIN;
    cfg.pirStuckSec  = DEF_PIR_STUCK_SEC;
    cfg.pirCoolSec   = DEF_PIR_COOL_SEC;
    cfg.pirMaxRunSec = DEF_PIR_MAX_RUN_SEC;
    settingsSanitize();
    settingsSave();
  } else if (cfg.magic != CFG_MAGIC) {
    settingsDefaults();
    settingsSanitize();
  } else {
    settingsSanitize();
  }
  // ключ района тримаємо в EEPROM, щоб під час TLS не читати flash
  if (cfg.alertKey[0] == 0) raionKey(cfg.alertRaion - 1, cfg.alertKey, sizeof(cfg.alertKey));
}

void settingsSave() {
  settingsSanitize();
  cfg.magic = CFG_MAGIC;
  EEPROM.put(EEPROM_CFG_ADDR, cfg);
  EEPROM.commit();
}

void loadCreds() {
  EEPROM.get(0, creds);
  creds.ssid[32] = '\0';
  creds.pass[63] = '\0';
  haveCreds = (creds.magic == CRED_MAGIC) && (creds.ssid[0] != '\0');
}

void saveCreds(const String& ssid, const String& pass) {
  creds.magic = CRED_MAGIC;
  memset(creds.ssid, 0, sizeof(creds.ssid));
  memset(creds.pass, 0, sizeof(creds.pass));
  ssid.substring(0, 32).toCharArray(creds.ssid, sizeof(creds.ssid));
  pass.substring(0, 63).toCharArray(creds.pass, sizeof(creds.pass));
  EEPROM.put(0, creds);
  EEPROM.commit();
  haveCreds = true;
}

void clearCreds() {
  creds.magic = 0;
  EEPROM.put(0, creds);
  EEPROM.commit();
  haveCreds = false;
}

// ---------- екранування ----------
String htmlEscape(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&':  o += "&amp;";  break;
      case '<':  o += "&lt;";   break;
      case '>':  o += "&gt;";   break;
      case '"':  o += "&quot;"; break;
      case '\'': o += "&#39;";  break;
      default:   o += c;
    }
  }
  return o;
}

String jsonEscape(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o;
}

// ---------- скан мереж (з кешем) ----------
void collectNets(int n) {
  netsAt = millis();
  netCount = 0;

  for (int i = 0; i < n && netCount < MAX_NETS; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;   // приховані без імені пропускаємо

    int32_t rssi = WiFi.RSSI(i);
    bool enc = (WiFi.encryptionType(i) != ENC_TYPE_NONE);

    int idx = -1;
    for (int j = 0; j < netCount; j++) {
      if (nets[j].ssid == s) { idx = j; break; }
    }
    if (idx >= 0) {
      if (rssi > nets[idx].rssi) nets[idx].rssi = rssi;   // mesh-дублікати
    } else {
      nets[netCount].ssid = s;
      nets[netCount].rssi = rssi;
      nets[netCount].enc = enc;
      netCount++;
    }
  }
  WiFi.scanDelete();

  // сортування за рівнем сигналу (сильніші вище)
  for (int i = 1; i < netCount; i++) {
    NetInfo t = nets[i];
    int j = i - 1;
    while (j >= 0 && nets[j].rssi < t.rssi) {
      nets[j + 1] = nets[j];
      j--;
    }
    nets[j + 1] = t;
  }

  netsValid = true;
}

void scanNets() {
  if (netsValid && (millis() - netsAt) < SCAN_TTL_MS) return;
  if (apScanPending) return;  // do not interrupt the portal's async scan
  collectNets(WiFi.scanNetworks(false, true));
}

// ================= ГРАФІК ВІДКЛЮЧЕНЬ =================
bool schedTimeOk() {
  return (uint32_t)time(nullptr) > 1700000000UL;   // після 2023-11 => NTP синхронізовано
}

bool schedDateMatches() {
  if (!schedValid || !schedTimeOk()) return false;
  time_t t = time(nullptr);
  struct tm lt;
  localtime_r(&t, &lt);
  return (lt.tm_mday == (int)schedDay) &&
         (lt.tm_mon + 1 == (int)schedMon) &&
         (lt.tm_year + 1900 == (int)schedYear);
}

int schedSlotForNow() {
  time_t t = time(nullptr);
  struct tm lt;
  localtime_r(&t, &lt);
  int s = lt.tm_hour * 2 + (lt.tm_min >= 30 ? 1 : 0);
  if (s < 0 || s >= SCHED_SLOTS) s = 0;
  return s;
}

bool schedComputeAllowed() {
  if (!cfg.schedEnabled) return true;          // фільтр вимкнено
  if (!schedDateMatches()) return cfg.schedFailOpen;  // немає даних на сьогодні
  int s = schedSlotForNow();
  return ((schedBits[s >> 3] >> (s & 7)) & 1) != 0;
}

// раз на секунду: оновлюємо кеш, щоб не смикати time() у циклі руху
void schedUpdate() {
  if (!cfg.schedEnabled) {
    schedAllowedNow = true;
    schedSlotOut = false;
    return;
  }
  schedSlotNow = schedSlotForNow();
  schedSlotOut = schedDateMatches()
    ? (((schedBits[schedSlotNow >> 3] >> (schedSlotNow & 7)) & 1) != 0)
    : false;
  schedAllowedNow = schedComputeAllowed();
}

const char* schedErrText() {
  switch (schedErr) {
    case 1:  return "немає Wi-Fi";
    case 2:  return "немає часу (NTP)";
    case 3:  return "не вдалося підключитися по HTTPS";
    case 4:  return "сервер віддав не 200";
    case 5:  return "групу не знайдено на сторінці";
    case 6:  return "не знайдено дату графіка";
    case 7:  return "не вибрано область або чергу";
    case 9:  return "порожня або пошкоджена відповідь джерела";
    default: return "невідома помилка";
  }
}

void schedTimeStr(uint32_t epoch, char* out) {
  if (!epoch || !schedTimeOk()) { strcpy(out, "--:--"); return; }
  time_t t = (time_t)epoch;
  struct tm lt;
  localtime_r(&t, &lt);
  sprintf(out, "%02d:%02d", lt.tm_hour, lt.tm_min);
}

static uint16_t schedRegionOffset(uint8_t idx) {
  uint16_t off = 0, lim = (uint16_t)(sizeof(SCHED_REGION_DATA) - 1);
  for (uint8_t i = 0; i < idx && off < lim; i++) {
    while (off < lim && pgm_read_byte(SCHED_REGION_DATA + off) != '\n') off++;
    off++;
  }
  return off;
}

// field 0 = назва, field 1 = HTTPS path
static void schedRegionField(uint8_t idx, uint8_t field, char* out, uint8_t cap) {
  uint16_t off = schedRegionOffset(idx), lim = (uint16_t)(sizeof(SCHED_REGION_DATA) - 1);
  if (field) while (off < lim && pgm_read_byte(SCHED_REGION_DATA + off) != '\t') off++;
  if (field && off < lim) off++;
  uint8_t n = 0; char c;
  while (off < lim) {
    c = (char)pgm_read_byte(SCHED_REGION_DATA + off++);
    if (c == '\n' || (!field && c == '\t') || c == 0) break;
    if (n + 1 < cap) out[n++] = c;
  }
  out[n] = 0;
}

static void schedRegionAppend(String& p, uint8_t idx) {
  char name[40]; schedRegionField(idx, 0, name, sizeof(name)); p += name;
}

// Обидва джерела розбираються потоково, без збереження відповіді в RAM.
bool schedFetchLegacyRaw();
bool schedFetchJsonRaw();
bool schedFetchMykolaivRaw();
bool schedFetchKirovohradRaw();
bool schedFetchSumyRaw();
bool schedFetchBezsvitlaRaw(uint8_t src, const char* queue);

bool schedFetch() {          // обгортка: позначаємо вікно Wi-Fi для фільтра PIR
  markRadioNoiseStart();
  bool ok = cfg.schedSource == 0 ? schedFetchLegacyRaw()
           : (cfg.schedSource == SCHED_SOURCE_MYKOLAIV ? schedFetchMykolaivRaw()
           : (cfg.schedSource == SCHED_SOURCE_KIROVOHRAD ? schedFetchKirovohradRaw()
           : (cfg.schedSource == SCHED_SOURCE_SUMY ? schedFetchSumyRaw() : schedFetchJsonRaw())));
  if (ok && !schedDateMatches()) {
    Serial.println(F("SCHED: графік не на сьогодні -> пробую дублер"));
    ok = false;
  }
  // якщо основне джерело не дало свіжих даних — беремо дублер bezsvitla.com.ua
  if (!ok && schedErr != 7 && schedErr != 1 && schedErr != 2) ok = schedFetchBezsvitlaRaw(cfg.schedSource, cfg.schedQueue);
  markRadioNoiseStop();
  return ok;
}

bool schedFetchLegacyRaw() {
  unsigned long started = millis();
  unsigned long deadline = started + SCHED_TIMEOUT_MS;
  uint32_t heapBefore = ESP.getFreeHeap();

  if (WiFi.status() != WL_CONNECTED) {
    schedErr = 1;
    schedNextAt = millis() + 5000UL;
    return false;
  }
  if (!schedTimeOk()) {
    schedErr = 2;
    schedNextAt = millis() + 5000UL;
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();                 // без перевірки сертифіката (економія RAM/флешу)
  client.setBufferSizes(16384, 1024);   // стандартний RX: Cloudflare шле великі записи
  client.setTimeout(15000);

  if (!client.connect(SCHED_HOST, 443)) {
    schedErr = 3;
    schedNextAt = millis() + SCHED_RETRY_MS;
    client.stop();
    Serial.printf("SCHED: connect failed, heap %u\n", ESP.getFreeHeap());
    return false;
  }

  client.print(F("GET " SCHED_PATH " HTTP/1.0\r\n"
                 "Host: " SCHED_HOST "\r\n"
                 "User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0 Safari/537.36\r\n"
                 "Accept: text/html,application/xhtml+xml\r\n"
                 "Accept-Encoding: identity\r\n"
                 "Connection: close\r\n\r\n"));

  auto rd = [&]() -> int {
    while (true) {
      if (client.available()) return client.read();
      if (!client.connected()) return -1;
      if (timeReached(millis(), deadline)) return -1;
      delay(1);
    }
  };

  // ---- заголовки ----
  char line[160];
  int li = 0;
  bool first = true;
  int httpCode = 0;
  while (true) {
    int c = rd();
    if (c < 0) break;
    if (c == '\n') {
      line[li] = 0;
      if (li > 0 && line[li - 1] == '\r') line[li - 1] = 0;
      if (first) {
        sscanf(line, "HTTP/%*d.%*d %d", &httpCode);
        Serial.printf("SCHED hdr [%s] code=%d\n", line, httpCode);
        first = false;
      } else if (li == 0) {
        break;   // кінець заголовків
      }
      li = 0;
    } else if (li < (int)sizeof(line) - 1) {
      line[li++] = (char)c;
    }
  }

  if (httpCode != 200) {
    schedErr = 4;
    schedNextAt = millis() + SCHED_RETRY_MS;
    client.stop();
    Serial.printf("SCHED: HTTP %d\n", httpCode);
    return false;
  }

  // ---- тіло ----
  const char *gneedle = "id=\"inf";
  const char *dneedle = "id=\"gsv_t\"";
  const int glen = 7, dlen = 10;

  int gm = 0, dm = 0;
  bool dateArmed = false, gotDate = false, got = false;
  char dbuf[10];
  int dbi = 0;
  uint8_t bits[6] = {0, 0, 0, 0, 0, 0};
  uint8_t dd = 0, dmon = 0;
  uint16_t dyear = 0;
  unsigned long total = 0;

  while (true) {
    int ci = rd();
    if (ci < 0) break;
    if (++total > SCHED_MAX_BYTES) break;
    char ch = (char)ci;

    // дата у #gsv_t: перші 10 цифр/крапок -> dd.mm.yyyy
    if (dateArmed) {
      if (ch >= '0' && ch <= '9') {
        if (dbi < 10) dbuf[dbi++] = ch;
        if (dbi == 10) {
          if (dbuf[2] == '.' && dbuf[5] == '.') {
            dd    = (dbuf[0] - '0') * 10 + (dbuf[1] - '0');
            dmon  = (dbuf[3] - '0') * 10 + (dbuf[4] - '0');
            dyear = (dbuf[6] - '0') * 1000 + (dbuf[7] - '0') * 100 +
                    (dbuf[8] - '0') * 10 + (dbuf[9] - '0');
            gotDate = true;
          }
          dateArmed = false;
        }
      } else if (ch == '.') {
        if (dbi < 10) dbuf[dbi++] = ch;
        if (dbi == 10) { /* хвіст: перевіримо нижче */ }
        if (dbi == 10 && dbuf[2] == '.' && dbuf[5] == '.') {
          dd    = (dbuf[0] - '0') * 10 + (dbuf[1] - '0');
          dmon  = (dbuf[3] - '0') * 10 + (dbuf[4] - '0');
          dyear = (dbuf[6] - '0') * 1000 + (dbuf[7] - '0') * 100 +
                  (dbuf[8] - '0') * 10 + (dbuf[9] - '0');
          gotDate = true;
          dateArmed = false;
        }
      } else if (dbi > 0) {
        dateArmed = false;
      }
    }

    if (!gotDate) {
      if (ch == dneedle[dm]) {
        if (++dm == dlen) {
          dm = 0;
          dateArmed = true;
          dbi = 0;
        }
      } else {
        dm = (ch == 'i') ? 1 : 0;
      }
    }

    // пошук id="infN"
    if (ch == gneedle[gm]) {
      if (++gm == glen) {
        gm = 0;
        int g = 0;
        int x;
        while ((x = rd()) >= 0) {
          if (x == '"') break;
          if (x >= '0' && x <= '9') g = g * 10 + (x - '0');
        }
        if (g == (int)cfg.schedGroup) {
          int idx = 0, y;
          while (idx < SCHED_SLOTS && (y = rd()) >= 0) {
            if (y == '<') {
              int z = rd();
              if (z < 0) break;
              // сайт позначає слот тегом: <u>з</u> = заживлені, <o>в</o> = відключені,
              // <s>мз</s> = можливо заживлені. Відключення = ТІЛЬКИ 'o'.
              if (z == 'u' || z == 'o' || z == 's') {
                if (z == 'o' || (z == 's' && cfg.schedSoft)) {
                  bits[idx >> 3] |= (uint8_t)(1 << (idx & 7));
                }
                idx++;
              }
            }
          }
          if (idx == SCHED_SLOTS) { got = true; break; }
        }
      }
    } else {
      gm = (ch == 'i') ? 1 : 0;
    }
  }

  client.stop();

  if (!got) {
    schedErr = 5;
    schedNextAt = millis() + SCHED_RETRY_MS;
  } else if (!gotDate) {
    schedErr = 6;
    schedNextAt = millis() + SCHED_RETRY_MS;
  } else {
    memcpy(schedBits, bits, sizeof(bits));
    schedDay = dd;
    schedMon = dmon;
    schedYear = dyear;
    schedValid = true;
    schedSyncEpoch = (uint32_t)time(nullptr);
    schedErr = 0;
    schedUpdate();
    schedNextAt = millis() + SCHED_REFRESH_MS;
  }

  Serial.printf("SCHED: grp=%u ok=%u err=%u date=%u.%02u.%u heap %u->%u (%lus)\n",
                cfg.schedGroup, got ? 1 : 0, schedErr, schedDay, schedMon, schedYear,
                heapBefore, ESP.getFreeHeap(), (millis() - started) / 1000UL);
  return got;
}

// Потоковий парсер безкоштовного JSON-формату:
// fact.data.<unix date>.<GPV queue>.{"1":"yes", ... "24":"no"}.
class ScheduleJsonSink : public Stream {
public:
  bool found = false;
  uint8_t bits[6] = {0};
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
  uint32_t len = 0;

  void reset(const char* queue, bool includePossible) {
    if (strncmp(queue, "GPV", 3) == 0) strncpy(wantQueue, queue, sizeof(wantQueue) - 1);
    else snprintf(wantQueue, sizeof(wantQueue), "GPV%s", queue);
    wantQueue[sizeof(wantQueue) - 1] = 0;
    usePossible = includePossible;
    found = false; len = 0; memset(bits, 0, sizeof(bits));
    curly = 0; inString = escaped = stringValue = afterColon = false;
    tokenLen = 0; lastKey[0] = 0; inToday = inQueue = false; hourCount = 0;
  }
  size_t write(uint8_t c) override { len++; feed((char)c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    len += n; for (size_t i = 0; i < n; i++) feed((char)b[i]); return n;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  char wantQueue[16] = {0};
  bool usePossible = false;
  int curly = 0;
  bool inString = false, escaped = false, stringValue = false;
  bool afterColon = false, inToday = false, inQueue = false;
  char token[24] = {0}, lastKey[24] = {0};
  uint8_t tokenLen = 0, hourCount = 0;

  void setSlot(int slot) {
    if (slot >= 0 && slot < 48) bits[slot >> 3] |= (uint8_t)(1 << (slot & 7));
  }
  void processHour() {
    int h = atoi(lastKey);
    if (h < 1 || h > 24) return;
    int first = (h - 1) * 2;
    bool maybe = strcmp(token, "maybe") == 0;
    bool mfirst = strcmp(token, "mfirst") == 0;
    bool msecond = strcmp(token, "msecond") == 0;
    if (strcmp(token, "no") == 0 || (usePossible && maybe)) { setSlot(first); setSlot(first + 1); }
    else if (strcmp(token, "first") == 0 || (usePossible && mfirst)) setSlot(first);
    else if (strcmp(token, "second") == 0 || (usePossible && msecond)) setSlot(first + 1);
    hourCount++;
  }
  void setDateFromEpoch(const char* s) {
    time_t t = (time_t)strtoul(s, nullptr, 10);
    struct tm dt, nowDt; time_t now = time(nullptr);
    localtime_r(&t, &dt); localtime_r(&now, &nowDt);
    inToday = dt.tm_year == nowDt.tm_year && dt.tm_yday == nowDt.tm_yday;
    if (inToday) { day = dt.tm_mday; mon = dt.tm_mon + 1; year = dt.tm_year + 1900; }
  }
  void finishString() {
    token[tokenLen] = 0;
    if (stringValue) {
      if (inQueue && curly == 5) processHour();
      afterColon = false;
    } else {
      strncpy(lastKey, token, sizeof(lastKey) - 1);
      lastKey[sizeof(lastKey) - 1] = 0;
    }
  }
  void feed(char c) {
    if (found) return;
    if (inString) {
      if (escaped) { escaped = false; if (tokenLen < sizeof(token)-1) token[tokenLen++] = c; return; }
      if (c == '\\') { escaped = true; return; }
      if (c == '"') { inString = false; finishString(); return; }
      if (tokenLen < sizeof(token)-1) token[tokenLen++] = c;
      return;
    }
    if (c == '"') {
      inString = true; escaped = false; tokenLen = 0;
      stringValue = afterColon;
      return;
    }
    if (c == '{') {
      if (curly == 3) setDateFromEpoch(lastKey);
      else if (curly == 4) {
        inQueue = inToday && strcmp(lastKey, wantQueue) == 0;
        if (inQueue) { memset(bits, 0, sizeof(bits)); hourCount = 0; }
      }
      curly++;
      afterColon = false; lastKey[0] = 0;
      return;
    }
    if (c == '}') {
      if (curly == 5 && inQueue) {
        found = (hourCount == 24);
        inQueue = false;
      }
      if (curly == 4) inToday = false;
      if (curly > 0) curly--;
      return;
    }
    if (c == ':') afterColon = true;
    else if (c == ',') { afterColon = false; lastKey[0] = 0; }
  }
};

bool schedFetchJsonRaw() {
  if (WiFi.status() != WL_CONNECTED) { schedErr = 1; schedNextAt = millis() + 5000UL; return false; }
  if (!schedTimeOk()) { schedErr = 2; schedNextAt = millis() + 5000UL; return false; }
  if (!cfg.schedQueue[0] || cfg.schedSource >= SCHED_REGION_COUNT) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  char path[128]; schedRegionField(cfg.schedSource, 1, path, sizeof(path));
  if (!path[0]) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);
  client.setTimeout(SCHED_TIMEOUT_MS);
  HTTPClient http;
  String url = F("https://" SCHED_JSON_HOST); url += path;
  if (!http.begin(client, url)) { schedErr = 3; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  http.setTimeout(SCHED_TIMEOUT_MS);
  http.addHeader(F("Accept"), F("application/json"));
  http.addHeader(F("User-Agent"), F("Svitlyak/2.0 ESP8266"));
  int code = http.GET();
  if (code != 200) {
    schedErr = (code > 0 ? 4 : 3);
    schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED JSON: HTTP %d\n", code); http.end(); return false;
  }
  ScheduleJsonSink sink;
  sink.reset(cfg.schedQueue, cfg.schedSoft != 0);
  int r = http.writeToStream(&sink);
  http.end();
  if (r < 0 || !sink.found) {
    schedErr = (sink.len == 0) ? 9 : 5;
    schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED JSON: parse failed r=%d len=%lu\n", r, (unsigned long)sink.len);
    return false;
  }
  memcpy(schedBits, sink.bits, sizeof(schedBits));
  schedDay = sink.day; schedMon = sink.mon; schedYear = sink.year;
  schedValid = true; schedSyncEpoch = (uint32_t)time(nullptr); schedErr = 0;
  schedUpdate(); schedNextAt = millis() + SCHED_REFRESH_MS;
  Serial.printf("SCHED JSON: region=%u queue=%s date=%u.%02u.%u len=%lu\n",
                cfg.schedSource, cfg.schedQueue, schedDay, schedMon, schedYear,
                (unsigned long)sink.len);
  return true;
}

static int mykolaivQueueId(const char* queue) {
  if (strncmp(queue, "GPV", 3) == 0) queue += 3;
  static const char* names[] = {"1.1","1.2","2.1","2.2","3.1","3.2","4.1","4.2","5.1","5.2","6.1","6.2"};
  static const uint8_t ids[] = {14,15,16,17,19,20,21,22,24,25,26,27};
  for (uint8_t i = 0; i < 12; i++) if (strcmp(queue, names[i]) == 0) return ids[i];
  return 0;
}

class MykolaivScheduleSink : public Stream {
public:
  bool found = false;
  uint8_t bits[6] = {0};
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
  uint32_t len = 0;

  void reset(uint8_t queueId, bool includePossible) {
    wantQueue = queueId; usePossible = includePossible;
    found = false; len = 0; memset(bits, 0, sizeof(bits));
    curly = 0; inString = escaped = stringValue = afterColon = numberActive = false;
    tokenLen = keyLen = 0; key[0] = 0; number = 0; inToday = false; resetCell();
  }
  size_t write(uint8_t c) override { len++; feed((char)c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    len += n; for (size_t i = 0; i < n; i++) feed((char)b[i]); return n;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  uint8_t wantQueue = 0;
  bool usePossible = false, inToday = false;
  int curly = 0;
  bool inString = false, escaped = false, stringValue = false, afterColon = false;
  bool numberActive = false;
  long number = 0;
  char token[32] = {0}, key[24] = {0};
  uint8_t tokenLen = 0, keyLen = 0;
  int cellQueue = 0, cellSlot = 0;
  uint8_t cellType = 0; // 1=OFF/SURE_OFF, 2=PROBABLY_OFF

  void resetCell() { cellQueue = cellSlot = 0; cellType = 0; }
  void setSlot(int slot) {
    if (slot >= 0 && slot < 48) bits[slot >> 3] |= (uint8_t)(1 << (slot & 7));
  }
  static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2; const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
  }
  void parseFrom() {
    if (tokenLen < 13 || token[4] != '-' || token[7] != '-' || token[10] != 'T') return;
    int y = atoi(token), m = atoi(token + 5), d = atoi(token + 8), h = atoi(token + 11);
    time_t epoch = (time_t)(daysFromCivil(y, m, d) * 86400LL + h * 3600LL);
    struct tm src, nowTm; time_t now = time(nullptr);
    localtime_r(&epoch, &src); localtime_r(&now, &nowTm);
    inToday = src.tm_year == nowTm.tm_year && src.tm_yday == nowTm.tm_yday;
    if (inToday) { day = nowTm.tm_mday; mon = nowTm.tm_mon + 1; year = nowTm.tm_year + 1900; }
  }
  void finishString() {
    token[tokenLen] = 0;
    if (stringValue) {
      if (curly == 1 && strcmp(key, "from") == 0) parseFrom();
      else if (curly == 2 && strcmp(key, "type") == 0) {
        if (strcmp(token, "OFF") == 0 || strcmp(token, "SURE_OFF") == 0) cellType = 1;
        else if (strcmp(token, "PROBABLY_OFF") == 0) cellType = 2;
      }
      afterColon = false;
    } else {
      strncpy(key, token, sizeof(key) - 1); key[sizeof(key) - 1] = 0; keyLen = tokenLen;
    }
  }
  void finishNumber() {
    if (!numberActive) return;
    if (curly == 2 && strcmp(key, "outage_queue_id") == 0) cellQueue = (int)number;
    else if (curly == 2 && strcmp(key, "time_series_id") == 0) cellSlot = (int)number;
    numberActive = false; afterColon = false;
  }
  void finishCell() {
    if (inToday && cellQueue == wantQueue && cellSlot >= 1 && cellSlot <= 48 &&
        (cellType == 1 || (cellType == 2 && usePossible))) setSlot(cellSlot - 1);
  }
  void feed(char c) {
    if (found) return;
    if (inString) {
      if (escaped) { escaped = false; if (tokenLen < sizeof(token)-1) token[tokenLen++] = c; return; }
      if (c == '\\') { escaped = true; return; }
      if (c == '"') { inString = false; finishString(); return; }
      if (tokenLen < sizeof(token)-1) token[tokenLen++] = c;
      return;
    }
    if (numberActive) {
      if (c >= '0' && c <= '9') { number = number * 10 + (c - '0'); return; }
      finishNumber();
    }
    if (c == '"') { inString = true; escaped = false; tokenLen = 0; stringValue = afterColon; return; }
    if (afterColon && c >= '0' && c <= '9') { numberActive = true; number = c - '0'; return; }
    if (c == '{') {
      curly++;
      if (curly == 1) { inToday = false; memset(bits, 0, sizeof(bits)); }
      else if (curly == 2) resetCell();
      afterColon = false; key[0] = 0; return;
    }
    if (c == '}') {
      if (curly == 2) finishCell();
      else if (curly == 1 && inToday) found = true;
      if (curly > 0) curly--;
      afterColon = false; return;
    }
    if (c == ':') afterColon = true;
    else if (c == ',') { afterColon = false; key[0] = 0; }
  }
};

bool schedFetchMykolaivRaw() {
  if (WiFi.status() != WL_CONNECTED) { schedErr = 1; schedNextAt = millis() + 5000UL; return false; }
  if (!schedTimeOk()) { schedErr = 2; schedNextAt = millis() + 5000UL; return false; }
  int queueId = mykolaivQueueId(cfg.schedQueue);
  if (!queueId) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  WiFiClientSecure client; client.setInsecure(); client.setBufferSizes(16384, 512); client.setTimeout(SCHED_TIMEOUT_MS);
  HTTPClient http;
  if (!http.begin(client, F("https://off.energy.mk.ua/api/v2/schedule/active"))) {
    schedErr = 3; schedNextAt = millis() + SCHED_RETRY_MS; return false;
  }
  http.setTimeout(SCHED_TIMEOUT_MS); http.addHeader(F("Accept"), F("application/json"));
  int code = http.GET();
  if (code != 200) {
    schedErr = code > 0 ? 4 : 3; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED MYK: HTTP %d\n", code); http.end(); return false;
  }
  MykolaivScheduleSink sink; sink.reset((uint8_t)queueId, cfg.schedSoft != 0);
  int r = http.writeToStream(&sink); http.end();
  if (r < 0 || !sink.found) {
    schedErr = sink.len ? 5 : 9; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED MYK: parse failed r=%d len=%lu\n", r, (unsigned long)sink.len); return false;
  }
  memcpy(schedBits, sink.bits, sizeof(schedBits));
  schedDay = sink.day; schedMon = sink.mon; schedYear = sink.year;
  schedValid = true; schedSyncEpoch = (uint32_t)time(nullptr); schedErr = 0;
  schedUpdate(); schedNextAt = millis() + SCHED_REFRESH_MS;
  Serial.printf("SCHED MYK: queue=%s date=%u.%02u.%u len=%lu\n",
                cfg.schedQueue, schedDay, schedMon, schedYear, (unsigned long)sink.len);
  return true;
}

// Кіровоградобленерго: офіційний графік публікується текстом у районних Telegram-каналах.
// Публічний веб-перегляд https://t.me/s/<channel> віддає останні ~20 постів звичайним HTML,
// без захисту й без JS. Кожен пост — увесь графік на свою дату, найновіший пост за добу перемагає.
class KirovohradScheduleSink : public Stream {
public:
  bool found = false;
  uint8_t bits[6] = {0};
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
  uint32_t len = 0;

  void reset(const char* queue) {
    if (strncmp(queue, "GPV", 3) == 0) queue += 3;
    strncpy(wantQueue, queue, sizeof(wantQueue) - 1); wantQueue[sizeof(wantQueue) - 1] = 0;
    found = false; len = 0; memset(bits, 0, sizeof(bits));
    inTag = inText = inEntity = false;
    tagLen = lineLen = entityLen = 0;
    tag[0] = 0; line[0] = 0; entity[0] = 0;
    postReset();
  }
  size_t write(uint8_t c) override { len++; feed((char)c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    len += n; for (size_t i = 0; i < n; i++) feed((char)b[i]); return n;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  char wantQueue[8] = {0};
  char tag[72] = {0}, line[120] = {0}, entity[8] = {0};
  uint8_t tagLen = 0, lineLen = 0, entityLen = 0;
  bool inTag = false, inText = false, inEntity = false;
  // стан одного поста
  uint8_t tmpBits[6] = {0}, pDay = 0, pMon = 0;
  uint16_t pYear = 0;
  bool tmpAny = false, postHasDate = false, cancelSeen = false, grafikSeen = false;

  void postReset() {
    memset(tmpBits, 0, sizeof(tmpBits));
    tmpAny = false; postHasDate = false; cancelSeen = grafikSeen = false;
    pDay = pMon = 0; pYear = 0; lineLen = 0; line[0] = 0;
  }
  static bool digits(const char*& p, int& v, int minN, int maxN) {
    v = 0; int n = 0;
    while (n < maxN && *p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; n++; }
    return n >= minN;
  }
  // Перша дата в пості — дата, на яку діє графік (DD.MM.YYYY або DD.MM.YY).
  void scanDate(const char* s) {
    for (const char* p = s; *p; p++) {
      const char* q = p;
      int d, m, y;
      if (!digits(q, d, 1, 2) || *q != '.') continue;
      q++;
      if (!digits(q, m, 1, 2) || *q != '.') continue;
      q++;
      int yn = 0;
      const char* y0 = q;
      if (!digits(q, y, 2, 4)) continue;
      yn = (int)(q - y0);
      if (yn == 2) y += 2000;
      if (d < 1 || d > 31 || m < 1 || m > 12 || y < 2024 || y > 2035) continue;
      pDay = (uint8_t)d; pMon = (uint8_t)m; pYear = (uint16_t)y;
      postHasDate = true;
      return;
    }
  }
  static int readHm(const char*& p) {
    while (*p == ' ') p++;
    if (*p < '0' || *p > '9') return -1;
    int h = 0, m = 0, n = 0;
    while (n < 2 && *p >= '0' && *p <= '9') { h = h * 10 + (*p - '0'); p++; n++; }
    if (*p == ':') {
      p++; n = 0;
      while (n < 2 && *p >= '0' && *p <= '9') { m = m * 10 + (*p - '0'); p++; n++; }
      if (n != 2) return -1;
    }
    if (h > 24 || m > 59) return -1;
    return h * 60 + m;
  }
  void handleLine() {
    line[lineLen] = 0;
    if (strstr(line, "скасовано")) cancelSeen = true;
    if (strstr(line, "рафік") || strstr(line, "ідключен")) grafikSeen = true;
    if (!postHasDate) scanDate(line);
    const char* p = strstr(line, "Черга");
    if (!p) return;
    p += 10;                                  // довжина "Черга" у UTF-8
    while (*p == ' ') p++;
    if (*p < '0' || *p > '9') return;
    char q[8]; uint8_t qi = 0;
    while ((*p == '.' || (*p >= '0' && *p <= '9')) && qi < sizeof(q) - 1) q[qi++] = *p++;
    q[qi] = 0;
    if (strcmp(q, wantQueue) != 0) return;
    while (*p == ' ') p++;
    if (*p != ':') return;
    p++;
    tmpAny = true;
    for (;;) {
      while (*p == ' ' || *p == ',' || *p == '\t') p++;
      if (!*p || *p == '-') break;            // "Черга 1.1: -" — без відключень
      int s = readHm(p);
      if (s < 0) break;
      while (*p == ' ') p++;
      if (*p != '-') break;
      p++;
      int e = readHm(p);
      if (e < 0) break;
      if (e == 0 && s > 0) e = 1440;
      if (e > s) {
        int first = s / 30, last = (e + 29) / 30;
        if (last > 48) last = 48;
        for (int k = first; k < last; k++) tmpBits[k >> 3] |= (uint8_t)(1 << (k & 7));
      }
    }
  }
  void endLine() { handleLine(); lineLen = 0; line[0] = 0; }
  void finishPost() {
    endLine();
    if (!postHasDate) { postReset(); return; }
    time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
    bool today = pYear == (uint16_t)(lt.tm_year + 1900) && pMon == (uint8_t)(lt.tm_mon + 1) && pDay == (uint8_t)lt.tm_mday;
    if (today && (tmpAny || (cancelSeen && grafikSeen))) {
      memcpy(bits, tmpBits, sizeof(bits));
      day = pDay; mon = pMon; year = pYear;
      found = true;
    }
    postReset();
  }
  void applyEntity() {
    entity[entityLen] = 0;
    char ch = 0;
    if (strcmp(entity, "nbsp") == 0) ch = ' ';
    else if (strcmp(entity, "quot") == 0) ch = '"';
    else if (strcmp(entity, "amp") == 0) ch = '&';
    if (ch && lineLen < sizeof(line) - 1) line[lineLen++] = ch;
    inEntity = false; entityLen = 0;
  }
  void feed(char c) {
    if (inEntity) {
      if (c == ';') applyEntity();
      else if (entityLen < sizeof(entity) - 1) entity[entityLen++] = c;
      else { inEntity = false; entityLen = 0; }
      return;
    }
    if (inTag) {
      if (c == '>') {
        inTag = false; tag[tagLen] = 0;
        if (!inText) {
          if (strstr(tag, "tgme_widget_message_text")) { inText = true; postReset(); }
        } else if (tag[0] == '/') {
          if (strncmp(tag, "/div", 4) == 0) { finishPost(); inText = false; }
        } else if (strncmp(tag, "br", 2) == 0) {
          endLine();
        }
        tagLen = 0; tag[0] = 0;
        return;
      }
      if (tagLen < sizeof(tag) - 1) tag[tagLen++] = c;
      else { inTag = false; tagLen = 0; }      // обірваний тег (JS на сторінці) — пропускаємо
      return;
    }
    if (c == '<') { inTag = true; tagLen = 0; return; }
    if (c == '&') { inEntity = true; entityLen = 0; return; }
    if (!inText) return;
    if (c == '\n' || c == '\r' || c == '\t') c = ' ';
    if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
  }
};

bool schedFetchKirovohradRaw() {
  if (WiFi.status() != WL_CONNECTED) { schedErr = 1; schedNextAt = millis() + 5000UL; return false; }
  if (!schedTimeOk()) { schedErr = 2; schedNextAt = millis() + 5000UL; return false; }
  if (!cfg.schedQueue[0]) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  WiFiClientSecure client; client.setInsecure(); client.setBufferSizes(16384, 512); client.setTimeout(SCHED_TIMEOUT_MS);
  HTTPClient http;
  if (!http.begin(client, F("https://t.me/s/SvitloKropyvnytskyiMisto"))) {
    schedErr = 3; schedNextAt = millis() + SCHED_RETRY_MS; return false;
  }
  http.setTimeout(SCHED_TIMEOUT_MS);
  http.addHeader(F("Accept"), F("text/html"));
  http.addHeader(F("Accept-Encoding"), F("identity"));
  int code = http.GET();
  if (code != 200) {
    schedErr = code > 0 ? 4 : 3; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED KIR: HTTP %d\n", code); http.end(); return false;
  }
  KirovohradScheduleSink sink; sink.reset(cfg.schedQueue);
  int r = http.writeToStream(&sink); http.end();
  if (r < 0 || !sink.found) {
    schedErr = sink.len ? 5 : 9; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED KIR: parse failed r=%d len=%lu\n", r, (unsigned long)sink.len); return false;
  }
  memcpy(schedBits, sink.bits, sizeof(schedBits));
  schedDay = sink.day; schedMon = sink.mon; schedYear = sink.year;
  schedValid = true; schedSyncEpoch = (uint32_t)time(nullptr); schedErr = 0;
  schedUpdate(); schedNextAt = millis() + SCHED_REFRESH_MS;
  Serial.printf("SCHED KIR: queue=%s date=%u.%02u.%u len=%lu\n",
                cfg.schedQueue, schedDay, schedMon, schedYear, (unsigned long)sink.len);
  return true;
}

// ---- Сумська область: офіційний сайт soe.com.ua ----
// Сторінка /consumers/shutdowns?tab=hourly-schedules тягне JSON з /next-api/shutdowns/schedules
// і розшифровує його в браузері: AES-256-GCM, перші 12 байт — IV, останні 16 — тег.
// Ключ будується з поточної дати, але перші 32 байти бере base-рядок + перша цифра року,
// тому фактично це сталий ключ. Розшифровуємо потоково (CTR), не тримаючи відповідь у RAM.
static const uint8_t AES_SBOX[256] PROGMEM = {
  0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
  0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
  0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
  0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
  0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
  0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
  0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
  0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
  0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
  0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
  0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
  0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
  0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
  0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
  0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
  0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static inline uint8_t aesXt(uint8_t a) { return (uint8_t)((a << 1) ^ ((a & 0x80) ? 0x1b : 0)); }
static uint8_t aesMul(uint8_t a, uint8_t b) {
  uint8_t r = 0;
  while (b) { if (b & 1) r ^= a; a = aesXt(a); b >>= 1; }
  return r;
}

struct Aes256 {
  uint8_t rk[240];
  void expand(const uint8_t* key) {
    static const uint8_t rc[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
    memcpy(rk, key, 32);
    uint8_t r = 0;
    for (uint8_t i = 8; i < 60; i++) {
      uint8_t a = rk[4 * (i - 1)], b = rk[4 * (i - 1) + 1];
      uint8_t c = rk[4 * (i - 1) + 2], d = rk[4 * (i - 1) + 3];
      if (i % 8 == 0) {
        uint8_t t = a;
        a = pgm_read_byte(&AES_SBOX[b]) ^ rc[r++];
        b = pgm_read_byte(&AES_SBOX[c]);
        c = pgm_read_byte(&AES_SBOX[d]);
        d = pgm_read_byte(&AES_SBOX[t]);
      } else if (i % 8 == 4) {
        a = pgm_read_byte(&AES_SBOX[a]); b = pgm_read_byte(&AES_SBOX[b]);
        c = pgm_read_byte(&AES_SBOX[c]); d = pgm_read_byte(&AES_SBOX[d]);
      }
      rk[4 * i]     = rk[4 * (i - 8)]     ^ a;
      rk[4 * i + 1] = rk[4 * (i - 8) + 1] ^ b;
      rk[4 * i + 2] = rk[4 * (i - 8) + 2] ^ c;
      rk[4 * i + 3] = rk[4 * (i - 8) + 3] ^ d;
    }
  }
  void encBlock(const uint8_t* in, uint8_t* out) const {
    uint8_t s[16], t[16];
    for (uint8_t i = 0; i < 16; i++) s[i] = in[i] ^ rk[i];
    for (uint8_t round = 1; round <= 14; round++) {
      for (uint8_t i = 0; i < 16; i++) s[i] = pgm_read_byte(&AES_SBOX[s[i]]);
      for (uint8_t c = 0; c < 4; c++)
        for (uint8_t r = 0; r < 4; r++) t[r + 4 * c] = s[r + 4 * ((c + r) & 3)];
      if (round != 14) {
        for (uint8_t c = 0; c < 4; c++) {
          uint8_t a0 = t[4 * c], a1 = t[4 * c + 1], a2 = t[4 * c + 2], a3 = t[4 * c + 3];
          s[4 * c]     = aesMul(a0, 2) ^ aesMul(a1, 3) ^ a2 ^ a3;
          s[4 * c + 1] = a0 ^ aesMul(a1, 2) ^ aesMul(a2, 3) ^ a3;
          s[4 * c + 2] = a0 ^ a1 ^ aesMul(a2, 2) ^ aesMul(a3, 3);
          s[4 * c + 3] = aesMul(a0, 3) ^ a1 ^ a2 ^ aesMul(a3, 2);
        }
      } else {
        memcpy(s, t, 16);
      }
      for (uint8_t i = 0; i < 16; i++) s[i] ^= rk[16 * round + i];
    }
    memcpy(out, s, 16);
  }
};

// Спершу тягнемо з відповіді рядок "payload" (base64), декодуємо його на льоту,
// розшифровуємо AES-CTR і віддаємо байти у парсер. Останні 16 байт (GCM-тег) відкидаються.
static const char SOE_GCM_KEY[] = "qasdw-enjenf-twfeft!!d451qwqw!-2";

class SoeScheduleSink : public Stream {
public:
  bool found = false;
  uint8_t bits[6] = {0};
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
  uint32_t len = 0;

  void reset(const char* queue) {
    if (strncmp(queue, "GPV", 3) == 0) queue += 3;
    strncpy(wantQueue, queue, sizeof(wantQueue) - 1); wantQueue[sizeof(wantQueue) - 1] = 0;
    found = false; len = 0; memset(bits, 0, sizeof(bits));
    needlePos = 0; phase = 0; b64Acc = 0; b64Bits = 0;
    ivLen = 0; ksPos = 16; delayLen = 0; haveToday = false;
    curly = 0; inString = escaped = stringValue = afterColon = false;
    tokenLen = 0; key[0] = 0; curQueue[0] = 0;
    memset(curBits, 0, sizeof(curBits));
    startMin = endMin = -1;
    cDay = cMon = 0; cYear = 0;
    aes.expand((const uint8_t*)SOE_GCM_KEY);
  }
  size_t write(uint8_t c) override { len++; feed((char)c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    len += n; for (size_t i = 0; i < n; i++) feed((char)b[i]); return n;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  char wantQueue[8] = {0};
  // зовнішній JSON
  uint8_t needlePos = 0, phase = 0;
  uint32_t b64Acc = 0;
  uint8_t b64Bits = 0;
  // GCM
  Aes256 aes;
  uint8_t iv[12] = {0}, ctr[16] = {0}, ks[16] = {0}, ksPos = 16;
  uint8_t ivLen = 0;
  uint8_t delay[16] = {0}, delayLen = 0;
  // внутрішній JSON
  int curly = 0, startMin = -1, endMin = -1;
  bool inString = false, escaped = false, stringValue = false, afterColon = false;
  bool haveToday = false;
  char token[48] = {0}, key[16] = {0};
  uint8_t tokenLen = 0;
  char curQueue[8] = {0};
  uint8_t curBits[6] = {0}, cDay = 0, cMon = 0;
  uint16_t cYear = 0;

  void setSlot(int slot) {
    if (slot >= 0 && slot < 48) curBits[slot >> 3] |= (uint8_t)(1 << (slot & 7));
  }
  static int hm(const char* s) {
    int h = atoi(s), m = 0;
    const char* p = strchr(s, ':');
    if (p) m = atoi(p + 1);
    if (h < 0 || h > 24 || m < 0 || m > 59) return -1;
    return h * 60 + m;
  }
  void applyInterval() {
    if (startMin < 0 || endMin < 0) return;
    int e = endMin;
    if (e == 0 && startMin > 0) e = 1440;
    if (e <= startMin || startMin >= 1440) return;
    int first = startMin / 30, last = (e + 29) / 30;
    if (last > 48) last = 48;
    for (int k = first; k < last; k++) setSlot(k);
  }
  void scanDate(const char* s) {
    for (const char* p = s; *p; p++) {
      if (*p < '0' || *p > '9') continue;
      int d = atoi(p);
      const char* q = p; while (*q >= '0' && *q <= '9') q++;
      if (*q != '.') continue;
      int m = atoi(q + 1);
      const char* r = q + 1; while (*r >= '0' && *r <= '9') r++;
      if (*r != '.') continue;
      int y = atoi(r + 1);
      if (d < 1 || d > 31 || m < 1 || m > 12 || y < 2024 || y > 2035) continue;
      cDay = (uint8_t)d; cMon = (uint8_t)m; cYear = (uint16_t)y;
      return;
    }
  }
  void finishString() {
    token[tokenLen] = 0;
    if (stringValue) {
      if (curly == 2) {
        if (strcmp(key, "cherga_n") == 0) { strncpy(curQueue, token, sizeof(curQueue) - 1); curQueue[sizeof(curQueue) - 1] = 0; }
        else if (strcmp(key, "last_update") == 0) scanDate(token);
      } else if (curly == 3) {
        if (strcmp(key, "start_time") == 0) startMin = hm(token);
        else if (strcmp(key, "end_time") == 0) endMin = hm(token);
      }
      afterColon = false;
    } else {
      strncpy(key, token, sizeof(key) - 1); key[sizeof(key) - 1] = 0;
    }
  }
  void plainByte(char c) {
    if (found) return;
    if (inString) {
      if (escaped) { escaped = false; if (tokenLen < sizeof(token) - 1) token[tokenLen++] = c; return; }
      if (c == '\\') { escaped = true; return; }
      if (c == '"') { inString = false; finishString(); return; }
      if (tokenLen < sizeof(token) - 1) token[tokenLen++] = c;
      return;
    }
    if (c == '"') { inString = true; escaped = false; tokenLen = 0; stringValue = afterColon; return; }
    if (c == '{') {
      curly++;
      if (curly == 2) { curQueue[0] = 0; memset(curBits, 0, sizeof(curBits)); cDay = cMon = 0; cYear = 0; }
      else if (curly == 3) startMin = endMin = -1;
      afterColon = false; key[0] = 0; return;
    }
    if (c == '}') {
      if (curly == 3) applyInterval();
      else if (curly == 2) {
        struct tm lt; time_t now = time(nullptr); localtime_r(&now, &lt);
        bool today = (cYear == 0) ||
                     (cYear == (uint16_t)(lt.tm_year + 1900) && cMon == (uint8_t)(lt.tm_mon + 1) && cDay == (uint8_t)lt.tm_mday);
        if (today && strcmp(curQueue, wantQueue) == 0) {
          memcpy(bits, curBits, sizeof(bits));
          if (cYear) { day = cDay; mon = cMon; year = cYear; }
          else { day = lt.tm_mday; mon = lt.tm_mon + 1; year = lt.tm_year + 1900; }
          found = true;
          haveToday = true;
        }
      }
      if (curly > 0) curly--;
      afterColon = false; key[0] = 0; return;
    }
    if (c == ':') { afterColon = true; return; }
    if (c == ',') { afterColon = false; key[0] = 0; return; }
  }
  void cipherByte(uint8_t b) {
    if (ivLen < 12) {
      iv[ivLen++] = b;
      if (ivLen == 12) { memcpy(ctr, iv, 12); ctr[12] = 0; ctr[13] = 0; ctr[14] = 0; ctr[15] = 2; ksPos = 16; }
      return;
    }
    if (ksPos >= 16) {
      aes.encBlock(ctr, ks); ksPos = 0;
      for (int8_t i = 15; i >= 12; i--) { if (++ctr[i]) break; }
    }
    uint8_t p = b ^ ks[ksPos++];
    if (delayLen < 16) delay[delayLen++] = p;
    else {
      char out = (char)delay[0];
      memmove(delay, delay + 1, 15);
      delay[15] = p;
      plainByte(out);
    }
  }
  void b64Push(char c) {
    uint8_t v;
    if (c >= 'A' && c <= 'Z') v = (uint8_t)(c - 'A');
    else if (c >= 'a' && c <= 'z') v = (uint8_t)(c - 'a' + 26);
    else if (c >= '0' && c <= '9') v = (uint8_t)(c - '0' + 52);
    else if (c == '+') v = 62;
    else if (c == '/') v = 63;
    else return;                                   // '=' та сміття пропускаємо
    b64Acc = (b64Acc << 6) | v;
    b64Bits += 6;
    if (b64Bits >= 8) {
      b64Bits -= 8;
      cipherByte((uint8_t)((b64Acc >> b64Bits) & 0xFF));
      b64Acc &= (1UL << b64Bits) - 1UL;
    }
  }
  void feed(char c) {
    static const char needle[] = "\"payload\":\"";
    if (phase == 0) {
      if (c == needle[needlePos]) {
        if (needle[++needlePos] == 0) { phase = 1; needlePos = 0; }
      } else {
        needlePos = (c == '"') ? 1 : 0;
      }
      return;
    }
    if (phase == 1) {
      if (c == '"') { phase = 2; return; }
      b64Push(c);
    }
  }
};

bool schedFetchSumyRaw() {
  if (WiFi.status() != WL_CONNECTED) { schedErr = 1; schedNextAt = millis() + 5000UL; return false; }
  if (!schedTimeOk()) { schedErr = 2; schedNextAt = millis() + 5000UL; return false; }
  if (!cfg.schedQueue[0]) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  WiFiClientSecure client; client.setInsecure(); client.setBufferSizes(16384, 512); client.setTimeout(SCHED_TIMEOUT_MS);
  HTTPClient http;
  if (!http.begin(client, F("https://www.soe.com.ua/next-api/shutdowns/schedules"))) {
    schedErr = 3; schedNextAt = millis() + SCHED_RETRY_MS; return false;
  }
  http.setTimeout(SCHED_TIMEOUT_MS);
  http.addHeader(F("Accept"), F("application/json"));
  http.addHeader(F("Accept-Encoding"), F("identity"));
  http.addHeader(F("X-Source"), F("soe-website"));
  http.addHeader(F("Referer"), F("https://www.soe.com.ua/consumers/shutdowns?tab=hourly-schedules"));
  int code = http.GET();
  if (code != 200) {
    schedErr = code > 0 ? 4 : 3; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED SUMY: HTTP %d\n", code); http.end(); return false;
  }
  SoeScheduleSink sink; sink.reset(cfg.schedQueue);
  int r = http.writeToStream(&sink); http.end();
  if (r < 0 || !sink.found) {
    schedErr = sink.len ? 5 : 9; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED SUMY: parse failed r=%d len=%lu\n", r, (unsigned long)sink.len); return false;
  }
  memcpy(schedBits, sink.bits, sizeof(schedBits));
  schedDay = sink.day; schedMon = sink.mon; schedYear = sink.year;
  schedValid = true; schedSyncEpoch = (uint32_t)time(nullptr); schedErr = 0;
  schedUpdate(); schedNextAt = millis() + SCHED_REFRESH_MS;
  Serial.printf("SCHED SUMY: queue=%s date=%u.%02u.%u len=%lu\n",
                cfg.schedQueue, schedDay, schedMon, schedYear, (unsigned long)sink.len);
  return true;
}

// ---- bezsvitla.com.ua: універсальне джерело-дублер ----
// Дані лежать у HTML сторінки черги: data-schedule-date="2026-11-18" і
// data-schedule-intervals="[{&quot;from&quot;:&quot;00:00&quot;,&quot;to&quot;:&quot;03:00&quot;,&quot;state&quot;:&quot;OFF&quot;},...]"
// Сторінка велика (~400 KB), тому розбираємо потоково і беремо лише інтервали OFF.
const char BEZSVITLA_HOST[] = "bezsvitla.com.ua";

// Слаг області для bezsvitla (порожній рядок = дублера немає).
// Порядок відповідає SCHED_REGION_DATA.
const char BEZSVITLA_SLUGS[] PROGMEM =
  "\n"                                   // 0 Чернівецька — офіційне джерело
  "\n"                                   // 1 м. Київ — сторінки немає
  "kyivska-oblast\n"                     // 2
  "dnipropetrovska-oblast\n"             // 3
  "odeska-oblast\n"                      // 4
  "vinnytska-oblast\n"                   // 5
  "cherkaska-oblast\n"                   // 6
  "chernihivska-oblast\n"                // 7
  "kharkivska-oblast\n"                  // 8
  "khmelnytska-oblast\n"                 // 9
  "\n"                                   // 10 Львівська — сайт графік не публікує
  "poltavska-oblast\n"                   // 11
  "\n"                                   // 12 Івано-Франківська — немає даних
  "\n"                                   // 13 Рівненська — немає даних
  "\n"                                   // 14 Тернопільська — немає даних
  "\n"                                   // 15 Закарпатська — немає даних
  "zaporizka-oblast\n"                   // 16
  "zhytomyrska-oblast\n"                 // 17
  "mykolaivska-oblast\n"                 // 18
  "kirovohradska-oblast\n"               // 19
  "sumska-oblast\n";                     // 20

static bool schedBezsvitlaSlug(uint8_t idx, char* out, size_t n) {
  size_t i = 0; uint8_t line = 0;
  while (true) {
    char c = (char)pgm_read_byte(&BEZSVITLA_SLUGS[i]);
    if (!c) return false;
    if (line == idx) {
      size_t k = 0;
      while (c && c != '\n') {
        if (k + 1 < n) out[k++] = c;
        i++; c = (char)pgm_read_byte(&BEZSVITLA_SLUGS[i]);
      }
      out[k] = 0;
      return k > 0;
    }
    if (c == '\n') line++;
    i++;
  }
}

class BezsvitlaScheduleSink : public Stream {
public:
  bool found = false;
  uint8_t bits[6] = {0};
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
  uint32_t len = 0;

  void reset() {
    found = false; len = 0; memset(bits, 0, sizeof(bits));
    day = mon = 0; year = 0;
    ph = 0; np = 0; dp = 0; dateBuf[0] = 0;
    inEnt = false; ent = 0;
    inStr = false; strValue = false; tokenLen = 0; token[0] = 0; key[0] = 0;
    fromMin = toMin = -1; isOff = false; intervals = 0;
  }
  size_t write(uint8_t c) override { len++; feed((char)c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    len += n; for (size_t i = 0; i < n; i++) feed((char)b[i]); return n;
  }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  uint8_t ph = 0;        // 0 шукаємо дату, 1 читаємо дату, 2 шукаємо інтервали, 3 розбір, 4 кінець
  uint8_t np = 0, dp = 0;
  char dateBuf[12] = {0};
  bool inEnt = false; uint8_t ent = 0;
  bool inStr = false, strValue = false;
  char token[8] = {0}; uint8_t tokenLen = 0;
  char key[8] = {0};
  int fromMin = -1, toMin = -1;
  bool isOff = false;
  uint16_t intervals = 0;

  static int hm(const char* s) {
    int h = 0, m = 0;
    const char* p = s;
    while (*p >= '0' && *p <= '9') h = h * 10 + (*p++ - '0');
    if (*p == ':') { p++; while (*p >= '0' && *p <= '9') m = m * 10 + (*p++ - '0'); }
    if (h > 24 || m > 59) return -1;
    return h * 60 + m;
  }
  void applyInterval() {
    intervals++;
    if (isOff && fromMin >= 0 && toMin >= 0) {
      int e = toMin;
      if (e == 0 && fromMin > 0) e = 1440;
      if (e > fromMin) {
        int first = fromMin / 30, last = (e + 29) / 30;
        if (last > SCHED_SLOTS) last = SCHED_SLOTS;
        for (int k = first; k < last; k++) bits[k >> 3] |= (uint8_t)(1 << (k & 7));
      }
    }
    fromMin = toMin = -1; isOff = false;
  }
  void jsonChar(char c) {
    if (inStr) {
      if (c == '"') {
        inStr = false; token[tokenLen] = 0;
        if (strValue) {
          if (!strcmp(key, "from")) fromMin = hm(token);
          else if (!strcmp(key, "to")) toMin = hm(token);
          else if (!strcmp(key, "state")) isOff = (token[0] == 'O' && token[1] == 'F');
          strValue = false;
        } else {
          strncpy(key, token, sizeof(key) - 1); key[sizeof(key) - 1] = 0;
        }
        return;
      }
      if (tokenLen < sizeof(token) - 1) token[tokenLen++] = c;
      return;
    }
    if (c == '"') { inStr = true; tokenLen = 0; return; }
    if (c == ':') { strValue = true; return; }
    if (c == '}') { applyInterval(); return; }
  }
  void feed(char c) {
    if (ph == 0) {
      static const char n0[] = "data-schedule-date=\"";
      if (c == n0[np]) { if (n0[++np] == 0) { ph = 1; np = 0; dp = 0; } }
      else np = (c == 'd') ? 1 : 0;
      return;
    }
    if (ph == 1) {
      if (c == '"') {
        dateBuf[dp] = 0;
        if (dp == 10 && dateBuf[4] == '-' && dateBuf[7] == '-') {
          year = (uint16_t)atoi(dateBuf);
          mon = (uint8_t)atoi(dateBuf + 5);
          day = (uint8_t)atoi(dateBuf + 8);
        }
        ph = 2; np = 0;
        return;
      }
      if (dp < sizeof(dateBuf) - 1) dateBuf[dp++] = c;
      return;
    }
    if (ph == 2) {
      static const char n2[] = "data-schedule-intervals=\"";
      if (c == n2[np]) { if (n2[++np] == 0) { ph = 3; np = 0; } }
      else np = (c == 'd') ? 1 : 0;
      return;
    }
    if (ph == 3) {
      if (inEnt) {                                // &quot; -> "
        static const char q[] = "quot;";
        if (c == q[ent]) { ent++; if (!q[ent]) { inEnt = false; ent = 0; jsonChar('"'); } }
        else { inEnt = false; ent = 0; }
        return;
      }
      if (c == '&') { inEnt = true; ent = 0; return; }
      if (c == '"') { ph = 4; found = (intervals > 0); return; }
      jsonChar(c);
    }
  }
};

bool schedFetchBezsvitlaRaw(uint8_t src, const char* queue) {
  char slug[28];
  if (!schedBezsvitlaSlug(src, slug, sizeof(slug))) return false;   // для цієї області дублера немає
  if (WiFi.status() != WL_CONNECTED) { schedErr = 1; schedNextAt = millis() + 5000UL; return false; }
  if (!schedTimeOk()) { schedErr = 2; schedNextAt = millis() + 5000UL; return false; }
  const char* q = queue;
  if (!q || !q[0]) { schedErr = 7; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  if (strncmp(q, "GPV", 3) == 0) q += 3;

  char path[64];
  int n = snprintf(path, sizeof(path), "/%s/cherha-", slug);
  for (const char* p = q; *p && n < (int)sizeof(path) - 1; ++p) path[n++] = (*p == '.') ? '-' : *p;
  path[n] = 0;

  WiFiClientSecure client; client.setInsecure(); client.setBufferSizes(16384, 512); client.setTimeout(SCHED_TIMEOUT_MS);
  HTTPClient http;
  String url = F("https://"); url += BEZSVITLA_HOST; url += path;
  if (!http.begin(client, url)) { schedErr = 3; schedNextAt = millis() + SCHED_RETRY_MS; return false; }
  http.setTimeout(SCHED_TIMEOUT_MS);
  http.addHeader(F("Accept"), F("text/html"));
  http.addHeader(F("Accept-Encoding"), F("identity"));
  http.addHeader(F("User-Agent"), F("Svitlyak/2.0 ESP8266"));
  int code = http.GET();
  if (code != 200) {
    schedErr = (code > 0 ? 4 : 3);
    schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED BEZ: HTTP %d %s\n", code, path); http.end(); return false;
  }
  BezsvitlaScheduleSink sink; sink.reset();
  int r = http.writeToStream(&sink);
  http.end();
  if (r < 0 || !sink.found) {
    schedErr = (sink.len == 0) ? 9 : 5;
    schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED BEZ: parse failed r=%d len=%lu\n", r, (unsigned long)sink.len);
    return false;
  }
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  if (sink.day != (uint8_t)lt.tm_mday || sink.mon != (uint8_t)(lt.tm_mon + 1) ||
      sink.year != (uint16_t)(lt.tm_year + 1900)) {
    schedErr = 5; schedNextAt = millis() + SCHED_RETRY_MS;
    Serial.printf("SCHED BEZ: дата %u.%02u.%u не сьогодні\n", sink.day, sink.mon, sink.year);
    return false;
  }
  memcpy(schedBits, sink.bits, sizeof(schedBits));
  schedDay = sink.day; schedMon = sink.mon; schedYear = sink.year;
  schedValid = true; schedSyncEpoch = (uint32_t)now; schedErr = 0;
  schedUpdate(); schedNextAt = millis() + SCHED_REFRESH_MS;
  Serial.printf("SCHED BEZ: %s queue=%s date=%u.%02u.%u len=%lu\n",
                slug, q, schedDay, schedMon, schedYear, (unsigned long)sink.len);
  return true;
}

// діагностика: /beztest?src=7&q=1.1 — примусово тягне графік з дублера
bool bezTestPending = false;
uint8_t bezTestSrc = 0;
char bezTestQ[16] = {0};

void schedTick() {
  static unsigned long lastCalc = 0;
  unsigned long now = millis();

  if (bezTestPending) {
    bezTestPending = false;
    Serial.printf("SCHED BEZTEST: src=%u q=%s\n", bezTestSrc, bezTestQ);
    schedFetchBezsvitlaRaw(bezTestSrc, bezTestQ);
  }

  if (schedForce || timeReached(now, schedNextAt)) {
    schedForce = false;
    if (!cfg.schedEnabled) {
      schedNextAt = now + SCHED_REFRESH_MS;   // нічого не тягнемо, поки вимкнено
    } else if (WiFi.status() == WL_CONNECTED) {
      schedFetch();
    } else {
      schedErr = 1;
      schedNextAt = now + 5000UL;
    }
  }

  if (now - lastCalc >= 1000) {
    lastCalc = now;
    schedUpdate();
  }
}

// ---------- повітряні тривоги: список районів ----------
// Запис: "key\tобласть - район\n". key = ключ API (lowercase UTF-8), збіг побайтовий.
// Читаємо з flash через pgm_read_byte() — вирівняні 32-бітні читання.
// RAM-копію НЕ тримаємо: вона з'їдає headroom, потрібний BearSSL (16 KB) для TLS.
static uint16_t raionOffset(uint16_t idx) {
  const uint16_t lim = (uint16_t)(sizeof(RAION_DATA) - 1);
  uint16_t off = 0;
  for (uint16_t i = 0; i < idx && off < lim; i++) {
    while (off < lim && pgm_read_byte(RAION_DATA + off) != '\n') off++;
    off++;
  }
  return (off > lim) ? lim : off;
}

// key района за індексом 0-based
void raionKey(uint16_t idx, char* buf, uint8_t max) {
  const uint16_t lim = (uint16_t)(sizeof(RAION_DATA) - 1);
  uint16_t off = raionOffset(idx);
  uint8_t i = 0;
  char c;
  while (off < lim) {
    c = (char)pgm_read_byte(RAION_DATA + off);
    if (c == '\t' || c == 0) break;
    off++;
    if (i + 1 < max) buf[i++] = c;
  }
  buf[i] = 0;
}

// дописати "область - район" запису idx (0-based) у рядок сторінки
void raionAppend(String& p, uint16_t idx) {
  const uint16_t lim = (uint16_t)(sizeof(RAION_DATA) - 1);
  uint16_t off = raionOffset(idx);
  while (off < lim && (char)pgm_read_byte(RAION_DATA + off) != '\t') off++;
  off++;
  char c;
  while (off < lim) {
    c = (char)pgm_read_byte(RAION_DATA + off);
    if (c == '\n' || c == 0) break;
    p += c;
    off++;
  }
}

const char* alertLevelName(uint8_t l) {
  return l == 2 ? "червоний" : (l == 1 ? "жовтий" : "немає тривоги");
}

const char* alertErrText() {
  switch (alertErr) {
    case 1:  return "немає Wi-Fi";
    case 3:  return "HTTPS-коннект не вдався";
    case 4:  return "сервер віддав не 200";
    default: return "ok";
  }
}

void alertNotifyStart(uint8_t color) {
  alertNotify = color;
  alertNotifyStartAt = millis();
  alertBlinkOn = false;
  lampState = ST_ALERT;
  Serial.print(F("ALERT notify: color="));
  Serial.println(color);
}

// новий рівень -> сповіщаємо ТІЛЬКИ на переході; останній рівень у EEPROM (не блимає двічі)
void alertApply(uint8_t lvl) {
  if (lvl == cfg.alertLastLevel) return;
  uint8_t old = cfg.alertLastLevel;
  cfg.alertLastLevel = lvl;
  settingsSave();
  Serial.print(F("ALERT: level "));
  Serial.print(old);
  Serial.print(F(" -> "));
  Serial.println(lvl);
  if (lvl > 0) alertNotifyStart(lvl);                       // жовтий/червоний
  else if (old > 0 && cfg.alertNotifyClear) alertNotifyStart(3);  // відбій -> зелений
}

// ---- потоковий сканер тіла відповіді ----
// Тіло (~9.5K) у RAM не тримаємо: HTTPClient віддає байти в цей Stream,
// сканер шукає "key":"<наш район>" і зчитує since/level.
// Пам'яті ~80 байт замість 9.5K, тому пік fetch більше не просідає до ~3K.
class AlertSink : public Stream {
public:
  bool     found = false;
  uint8_t  lvl   = 0;
  char     since[6] = {0};
  uint32_t len   = 0;

  void reset(const char* wantKey) {
    want    = wantKey;
    wantLen = (uint8_t)strlen(wantKey);
    found = false; lvl = 0; len = 0; since[0] = 0;
    st = ST_KEY; keyLen = 0; keyOk = false;
    winLen = 0; dateLen = 0; gotLevel = false; gotSince = false;
  }

  size_t write(uint8_t c) override { len++; feed(c); return 1; }
  size_t write(const uint8_t* buf, size_t n) override {
    len += n;
    for (size_t i = 0; i < n; i++) feed(buf[i]);
    return n;
  }
  int  available() override { return 0; }
  int  read() override { return -1; }
  int  peek() override { return -1; }
  void flush() override {}
  // Критично: Stream::sendSize пише лише коли availableForWrite() > 0
  // (у Print за замовчуванням 0 -> sendSize нічого не пише і падає в таймаут).
  int  availableForWrite() override { return 512; }
  bool outputCanTimeout() override { return false; }

private:
  enum { ST_KEY, ST_KEYREAD, ST_FIELDS, ST_LEVELCH, ST_SINCE };
  const char* want = nullptr;
  uint8_t wantLen = 0;
  uint8_t st = ST_KEY;
  uint8_t win[12];
  uint8_t winLen = 0;
  uint8_t keyLen = 0;
  bool    keyOk = false;
  bool    gotLevel = false, gotSince = false;
  uint8_t dateLen = 0;
  char    dateBuf[20] = {0};

  void push(uint8_t c) {
    if (winLen < sizeof(win)) { win[winLen++] = c; return; }
    memmove(win, win + 1, sizeof(win) - 1);
    win[sizeof(win) - 1] = c;
  }
  bool tailIs(const char* p, uint8_t n) const {
    if (winLen < n) return false;
    return memcmp(win + winLen - n, p, n) == 0;
  }

  void feed(uint8_t c) {
    push(c);
    if (found && gotLevel && gotSince) return;   // усе знайшли — решту не парсимо

    switch (st) {
      case ST_KEY:
        if (tailIs("\"key\":\"", 7)) { st = ST_KEYREAD; keyLen = 0; keyOk = true; }
        break;

      case ST_KEYREAD:
        if (c == '"') {
          if (keyOk && keyLen == wantLen) { found = true; gotLevel = gotSince = false; st = ST_FIELDS; }
          else { st = ST_KEY; winLen = 0; }
        } else if (keyOk) {
          if (keyLen < wantLen && (char)c == want[keyLen]) keyLen++;
          else keyOk = false;
        }
        break;

      case ST_FIELDS:   // у цьому API порядок: key, name, oblast, since, level
        if (!gotSince && tailIs("\"since\":\"", 9)) { st = ST_SINCE; dateLen = 0; }
        else if (!gotLevel && tailIs("\"level\":\"", 9)) { st = ST_LEVELCH; }
        break;

      case ST_LEVELCH:
        if (c == 'r') lvl = 2;
        else if (c == 'y') lvl = 1;
        gotLevel = true;
        st = ST_FIELDS;
        break;

      case ST_SINCE:
        if (dateLen < sizeof(dateBuf) - 1) dateBuf[dateLen++] = (char)c;
        if (dateLen >= 16) {   // 2026-11-16T03:57:07 -> HH=[11..12], MM=[14..15]
          since[0] = dateBuf[11]; since[1] = dateBuf[12]; since[2] = ':';
          since[3] = dateBuf[14]; since[4] = dateBuf[15]; since[5] = 0;
          gotSince = true;
          st = ST_FIELDS;
        }
        break;
    }
  }
};

// Звичайний GET (HTTPClient) + потоковий розбір тіла по нашому ключу.
void alertFetchRaw();

void alertFetch() {          // обгортка: позначаємо вікно Wi-Fi для фільтра PIR
  markRadioNoiseStart();
  alertFetchRaw();
  markRadioNoiseStop();
}

void alertFetchRaw() {
  unsigned long started = millis();
  uint32_t heapBefore = ESP.getFreeHeap();
  alertErr = 0;

  // Пік fetch тепер: TLS RX 16.4K + handshake ~8K + сканер ~0 = ~25K.
  // При старті 34K dip лишається >= 8K.
  if (ESP.getFreeHeap() < 34000UL) {
    alertNextAt = millis() + 10000UL;
    Serial.println(F("ALERT: heap low, skip"));
    return;
  }

  // RAM-ключ з EEPROM: flash під час TLS не читаємо
  char wantBuf[32];
  if (cfg.alertKey[0] == 0) raionKey(cfg.alertRaion - 1, cfg.alertKey, sizeof(cfg.alertKey));
  strncpy(wantBuf, cfg.alertKey, 31);
  wantBuf[31] = 0;

  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);   // "Minimum safe" ядра: RX має вмістити TLS-запис цілком
  client.setTimeout(15000);            // 15 с: більший таймаут на цьому ядрі залипає (wdt/hang)

  HTTPClient http;
  http.begin(client, F("https://" ALERT_HOST ALERT_PATH));
  http.setTimeout(15000);
  http.addHeader("User-Agent", ALERT_UA);
  http.addHeader("Accept", "application/json");

  int code = http.GET();
  if (code != 200) {
    alertErr = (code > 0) ? 4 : 3;
    alertNextAt = millis() + ALERT_RETRY_MS;
    http.end();
    Serial.print(F("ALERT: HTTP "));
    Serial.print(code);
    Serial.print(F(" heap "));
    Serial.println(ESP.getFreeHeap());
    return;
  }

  AlertSink sink;
  sink.reset(wantBuf);
  int r = http.writeToStream(&sink);   // HTTPClient сам розбирає Transfer-Encoding: chunked
  http.end();

  if (r < 0 || sink.len == 0) {
    alertErr = 5;
    alertNextAt = millis() + ALERT_RETRY_MS;
    Serial.print(F("ALERT: empty body r="));
    Serial.print(r);
    Serial.print(F(" len="));
    Serial.print(sink.len);
    Serial.print(F(" heap "));
    Serial.print(ESP.getFreeHeap());
    Serial.print(F(" ("));
    Serial.print((millis() - started) / 1000UL);
    Serial.println(F("s)"));
    return;
  }

  uint8_t lvl = sink.lvl;
  bool found = sink.found;
  alertSince[0] = 0;
  if (sink.since[0]) memcpy(alertSince, sink.since, 6);

  alertValid = true;
  alertLevelNow = lvl;
  alertCheckAt = millis();
  alertNextAt = millis() + (uint32_t)cfg.alertPollSec * 1000UL;
  if (cfg.alertEnabled) alertApply(lvl);

  Serial.print(F("ALERT: r="));
  Serial.print(cfg.alertRaion);
  Serial.print(F(" lvl="));
  Serial.print(lvl);
  Serial.print(F(" err="));
  Serial.print(alertErr);
  Serial.print(F(" since="));
  Serial.print(alertSince);
  Serial.print(F(" heap "));
  Serial.print(heapBefore);
  Serial.print(F("->"));
  Serial.print(ESP.getFreeHeap());
  Serial.print(F(" ("));
  Serial.print((millis() - started) / 1000UL);
  Serial.print(F("s) found="));
  Serial.print(found ? 1 : 0);
  Serial.print(F(" len="));
  Serial.println(sink.len);
}
void alertTick() {
  unsigned long now = millis();

  if (alertForce || timeReached(now, alertNextAt)) {
    alertForce = false;
    if (!cfg.alertEnabled) {
      alertNextAt = now + 60000UL;   // нічого не тягнемо, поки вимкнено
    } else if (WiFi.status() == WL_CONNECTED) {
      alertFetch();
    } else {
      alertErr = 1;
      alertNextAt = now + 5000UL;
    }
  }

  if (alertNotify && timeReached(now, alertNotifyStartAt + ALERT_NOTIFY_MS)) {
    alertNotify = 0;
    lampState = ST_OFF;
    lampWhy = "alert-end"; lampChanges++;
    lampOffCooldown(now);
    pirBlindFor(1200);        // мотор ще «дзвенить» -> не рахуємо це рухом
    motionUntil = 0;
    digitalWrite(PIN_VIB, LOW);
    vibActive = false;
  }
}

// ---------- сторінки ----------
void addNumField(String& p, const char* label, const char* name, int value, int lo, int hi) {  p += F("<div class='row'><label>");
  p += label;
  p += F("</label><input type='number' name='");
  p += name;
  p += F("' value='");
  p += String(value);
  p += F("' min='");
  p += String(lo);
  p += F("' max='");
  p += String(hi);
  p += F("'></div>");
}

// Потокова (chunked) віддача сторінки через публічний sendContent:
// один рядок ~8 KB на фрагментованому heap не виділяється -> crash.
static void httpChunkHeader() {
  server.sendContent(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                       "Transfer-Encoding: chunked\r\nConnection: close\r\n\r\n"));
}
static void httpChunk(const String& s) {
  if (s.length() == 0) return;
  String f = String(s.length(), HEX) + F("\r\n") + s + F("\r\n");
  server.sendContent(f);
}
static void httpChunkEnd() {
  server.sendContent(F("0\r\n\r\n"));
}

bool requestFromAP() {
  // Check the interface receiving this connection, not just whether AP is running.
  return apRunning && server.client().localIP() == WiFi.softAPIP();
}

void redirectToPortal() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "Open the Wi-Fi setup portal");
}

void handleAPNetworks() {
  server.sendHeader("Cache-Control", "no-store");
  if (!requestFromAP()) { server.send(403, "text/plain", "AP only"); return; }
  if (apScanPending) {
    const int result = WiFi.scanComplete();
    if (result == WIFI_SCAN_RUNNING) {
      server.send(202, "application/json", "{\"scanning\":true}"); return;
    }
    apScanPending = false;
    if (result < 0) {
      WiFi.scanDelete();
      server.send(503, "application/json", "{\"error\":\"scan_failed\"}"); return;
    }
    collectNets(result);
  } else if (connectPending && millis() - connectStartedAt < WIFI_TIMEOUT) {
    // Scanning during a deliberate join can move the AP radio off-channel.
    server.send(202, "application/json", "{\"scanning\":true}"); return;
  } else if (!netsValid || server.hasArg("refresh") || millis() - netsAt >= AP_SCAN_TTL_MS) {
    const int result = WiFi.scanNetworks(true, false);
    if (result == WIFI_SCAN_RUNNING) {
      apScanPending = true;
      server.send(202, "application/json", "{\"scanning\":true}"); return;
    }
    if (result < 0) {
      server.send(503, "application/json", "{\"error\":\"scan_failed\"}"); return;
    }
    collectNets(result);
  }
  String json = "{\"networks\":[";
  for (int i = 0; i < netCount; ++i) {
    if (i) json += ',';
    json += "{\"ssid\":\"" + jsonEscape(nets[i].ssid) + "\",\"rssi\":" + String(nets[i].rssi);
    json += ",\"secure\":"; json += nets[i].enc ? "true}" : "false}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void pageAPWiFi() {
  // Send the small page immediately; scanning must not stall captive-portal probes.
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send_P(200, "text/html; charset=utf-8", PSTR(R"HTML(<!doctype html>
<html lang="uk"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Світляк — підключення Wi-Fi</title><style>
body{font:17px system-ui,Arial;background:#0f141a;color:#e8eef5;max-width:420px;margin:32px auto;padding:18px;color-scheme:dark}
form{padding:20px;background:#1a222b;border-radius:16px}h1{font-size:25px}label{display:block;margin-top:14px}
select,input,button{box-sizing:border-box;width:100%;padding:13px;margin:8px 0;border-radius:9px;border:1px solid #33414f;font:inherit}
button{background:#2f81f7;color:white;cursor:pointer}button:disabled{opacity:.5}small,p{line-height:1.5}a{color:#79b8ff}
</style></head><body><h1>Підключити Світляк до Wi-Fi</h1>
<form method="post" action="/connect" id="wifiForm">
<label for="ssid">Домашня мережа 2,4 ГГц</label><select id="ssid" name="ssid" required disabled><option value="">Шукаємо мережі…</option></select>
<button id="refresh" type="button">Оновити список мереж</button><small id="scanHint" role="status"></small>
<label for="pass">Пароль Wi-Fi</label><input id="pass" name="pass" type="password" maxlength="63" autocomplete="new-password" placeholder="Пароль домашньої мережі">
<button id="connect" disabled>Підключити</button></form>
<p id="result" role="status">Вибери свою мережу та введи пароль. Налаштування лампи будуть доступні після підключення.</p>
<small>Якщо телефон пише «без інтернету», залишайся в мережі Світляка. Адреса цієї сторінки: <b>http://192.168.4.1/</b></small>
<script>
const ssid=document.getElementById('ssid'),refresh=document.getElementById('refresh'),hint=document.getElementById('scanHint'),connect=document.getElementById('connect');
const result=document.getElementById('result');
async function request(path){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),8000);
try{const response=await fetch(path,{cache:'no-store',signal:controller.signal});return {status:response.status,ok:response.ok,data:await response.json()};}finally{clearTimeout(timer);}}
async function scan(force=false){refresh.disabled=true;ssid.disabled=true;connect.disabled=true;hint.textContent='Шукаємо мережі…';const previous=ssid.value;
try{for(let attempt=0;attempt<30;attempt++){
const response=await request(force&&attempt===0?'/wifi-networks?refresh=1':'/wifi-networks');
if(response.status===202){await new Promise(resolve=>setTimeout(resolve,1000));continue;}
if(!response.ok)throw Error();const data=response.data;ssid.replaceChildren(new Option('Вибери свою мережу',''));
for(const net of data.networks)ssid.add(new Option(net.ssid+(net.secure?' · 🔒':' · відкрита')+' · '+net.rssi+' dBm',net.ssid));
if([...ssid.options].some(option=>option.value===previous))ssid.value=previous;
hint.textContent=data.networks.length?'Назву мережі вводити не потрібно.':'Мереж не знайдено. Перевір, чи ввімкнений Wi-Fi 2,4 ГГц, та онови список.';return;
}throw Error();}catch(e){hint.textContent='Не вдалося отримати список. Натисни «Оновити список мереж».';}
finally{refresh.disabled=false;ssid.disabled=false;connect.disabled=!ssid.value;}}
ssid.onchange=()=>{connect.disabled=!ssid.value;};refresh.onclick=()=>scan(true);
document.getElementById('wifiForm').onsubmit=()=>{connect.disabled=true;connect.textContent='Підключення…';};
async function check(){try{const s=(await request('/status')).data;if(s.state==='ok'){
result.replaceChildren(document.createTextNode('Підключено! Поверни телефон до домашньої мережі. Сторінка лампи: '));
const link=document.createElement('a');link.href='http://'+s.ip+'/';link.textContent=link.href;result.appendChild(link);return;}}
catch(e){}setTimeout(check,2000);}scan();check();
</script></body></html>)HTML"));
}

void lanRoute(const char* path, HTTPMethod method, ESP8266WebServer::THandlerFunction handler) {
  server.on(path, method, [handler]() {
    if (requestFromAP()) { redirectToPortal(); return; }
    handler();
  });
}

void handleRoot() {
  if (requestFromAP()) { pageAPWiFi(); return; }
  if (server.hasArg("rescan")) netsValid = false;
  scanNets();

  // Перший запуск (або «запустити майстер знову») -> покрокове налаштування
  if (!cfg.setupDone || server.hasArg("wiz")) {
    pageWizard();
    return;
  }

  String p;
  p.reserve(1000);
  httpChunkHeader();
  auto out = [&]() {
    httpChunk(p);
    p = "";
  };

  p += F(R"HTML(<!DOCTYPE html><html lang='uk'><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Нічник — налаштування</title><style>
body{margin:0;padding:16px;font-family:system-ui,Arial,sans-serif;background:#0f141a;color:#e8eef5;color-scheme:dark}
h2{margin:0 0 14px;font-size:20px}h3{margin:0 0 12px;font-size:15px;color:#9fb3c8;font-weight:600}
.card{background:#1a222b;border-radius:12px;padding:14px;margin-bottom:14px}
.row{display:flex;align-items:center;gap:10px;margin:9px 0}
.row label{flex:1;font-size:14px;color:#c9d6e3}
.row input[type=number],.row select{width:118px;padding:9px;border-radius:8px;border:1px solid #33414f;background:#0d1218;color:#fff;font-size:15px;text-align:right}
.net{display:flex;align-items:center;gap:10px;padding:10px;border-radius:8px;margin:4px 0;background:#212c38}
.net small{margin-left:auto;color:#8fa3b8;white-space:nowrap}
input[type=password]{width:100%;padding:12px;border-radius:8px;border:1px solid #33414f;background:#0d1218;color:#fff;font-size:16px;box-sizing:border-box}
button{width:100%;padding:14px;margin-top:12px;border:0;border-radius:10px;background:#2f81f7;color:#fff;font-size:16px;font-weight:600}
button.ghost{background:#2a3542}button.danger{background:#a8341f}
a{color:#79b8ff}
.stat{display:flex;flex-wrap:wrap;gap:8px}
.stat div{background:#212c38;border-radius:8px;padding:8px 10px;font-size:14px}
.stat b{color:#fff}
.row input[type=checkbox]{width:24px;height:24px;accent-color:#2f81f7}
.schedbar{display:flex;gap:2px;margin:10px 0}
.schedbar i{flex:1;height:16px;border-radius:2px;background:#2a3542}
.schedbar i.b{background:#dc7463}
.schedbar i.g{background:#4a7a4a}
.hint{font-size:12px;color:#8fa3b8;margin-top:6px}
.field{margin:14px 0}
.field label{display:block;font-size:13px;color:#9fb3c8;margin-bottom:6px}
.field select,.field input[type=text],.field input[type=password]{width:100%;padding:12px;border-radius:10px;border:1px solid #33414f;background:#0d1218;color:#fff;font-size:16px;box-sizing:border-box}
.field input[type=range]{width:100%;accent-color:#2f81f7;height:28px}
.field.toggle{display:flex;align-items:center;justify-content:space-between;gap:12px}
.field.toggle label{margin:0}
input[type=checkbox]{width:26px;height:26px;accent-color:#2f81f7}
details.card summary{cursor:pointer;font-size:15px;color:#9fb3c8;font-weight:600;list-style:none}
details.card summary::-webkit-details-marker{display:none}
details.card summary:before{content:'▸ ';color:#5b7089}
details.card[open] summary:before{content:'▾ '}
.steps{display:flex;gap:6px;margin:0 0 14px}
.steps i{flex:1;height:5px;border-radius:3px;background:#2a3542}
.steps i.on{background:#2f81f7}
.big{font-size:17px;font-weight:700;color:#fff}
</style></head><body><h2>Нічник — налаштування</h2>
<div class='card'><h3>Стан зараз</h3><div class='stat'>
<div>Режим: <b id='s_state'>—</b></div>
<div>Освітленість: <b id='s_lvl'>—</b> <small>(<span id='s_ldr'>—</span>)</small></div>
<div>Яскравість: <b id='s_br'>—</b></div>
<div>PIR: <b id='s_pir'>—</b></div>
<div>День: <b id='s_day'>—</b></div>
<div>Графік: <b id='s_sched'>—</b></div>
<div>Тривога: <b id='s_alert'>—</b></div>
<div>Wi-Fi: <b id='s_wifi'>—</b></div>
<div>AP: <b id='s_ap'>—</b></div>
<div>Працює: <b id='s_up'>—</b></div>
</div><div class='hint' style='margin-top:12px'><a href='/pir'>Відкрити живу діагностику PIR →</a></div>)HTML");
  out();

  if (server.hasArg("saved")) {
    p += F("<div class='card' style='border:1px solid #2f6f3f'>✅ Налаштування збережено</div>");
  }

  // ================= ПРОСТІ НАЛАШТУВАННЯ (для покупця) =================
  p += F("<form method='POST' action='/save'><div class='card'><h3>Як має працювати нічник</h3>");

  p += F("<div class='field'><label>Коли вмикати світло</label><select name='pirMode'>"
         "<option value='motion'");
  if (cfg.pirEnabled) p += F(" selected");
  p += F(">Коли хтось рухається (датчик руху)</option><option value='always'");
  if (!cfg.pirEnabled) p += F(" selected");
  p += F(">Світити всю ніч, щойно стемніє</option></select></div>");

  p += F("<div class='field'><label>Яскравість у темряві — <b id='brv'>");
  p += String((int)cfg.maxBright * 100 / 255);
  p += F("%</b></label><input type='range' name='maxBright' min='20' max='255' step='5' value='");
  p += String(cfg.maxBright);
  p += F("' oninput=\"document.getElementById('brv').textContent=Math.round(this.value*100/255)+'%'\"></div>");

  p += F("<div class='field'><label>Скільки світити після руху</label><select name='holdSec'>");
  const int HOLD_OPTS[8] = {5, 10, 15, 20, 30, 60, 120, 300};
  bool holdListed = false;
  for (int i = 0; i < 8; i++) if ((int)cfg.holdSec == HOLD_OPTS[i]) holdListed = true;
  if (!holdListed) {
    p += F("<option value='");
    p += String(cfg.holdSec);
    p += F("' selected>");
    p += String(cfg.holdSec);
    p += F(" с</option>");
  }
  for (int i = 0; i < 8; i++) {
    p += F("<option value='");
    p += String(HOLD_OPTS[i]);
    p += F("'");
    if ((int)cfg.holdSec == HOLD_OPTS[i]) p += F(" selected");
    p += F(">");
    p += String(HOLD_OPTS[i]);
    p += F(" секунд</option>");
  }
  p += F("</select><div class='hint'>Поки PIR тримає HIGH — світло не гасне. Цей час відраховується після останнього HIGH.</div></div>");

  p += F("<div class='field'><label>Вмикатись, коли надворі</label><select name='nightArm'>");
  const int DARK_OPTS[3] = {300, 390, 480};
  const char* DARK_NAMES[3] = {"вже сутеніє", "темно", "зовсім темно"};
  bool darkListed = false;
  for (int i = 0; i < 3; i++) if ((int)cfg.nightArm == DARK_OPTS[i]) darkListed = true;
  if (!darkListed) {
    p += F("<option value='");
    p += String(cfg.nightArm);
    p += F("' selected>власне значення (");
    p += String(cfg.nightArm);
    p += F(")</option>");
  }
  for (int i = 0; i < 3; i++) {
    p += F("<option value='");
    p += String(DARK_OPTS[i]);
    p += F("'");
    if ((int)cfg.nightArm == DARK_OPTS[i]) p += F(" selected");
    p += F(">");
    p += DARK_NAMES[i];
    p += F("</option>");
  }
  p += F("</select></div>");
  p += F("<div class='hint'>Якщо вночі лампа не реагує на рух — виберіть «зовсім темно».</div>");
  p += F("</div>");
  out();

  // ---- графік відключень (опційно) ----
  p += F("<div class='card'><h3>Світло за графіком відключень</h3>"
         "<div class='field toggle'><label>Світити лише тоді, коли у вас вимкнено електрику</label>"
         "<input type='checkbox' name='schedEnabled' value='1'");
  if (cfg.schedEnabled) p += F(" checked");
  p += F("></div>");

  p += F("<div class='field'><label>Область / оператор</label><select name='schedSource'>");
  for (int r = 0; r < SCHED_REGION_COUNT; r++) {
    p += F("<option value='"); p += String(r); p += F("'");
    if ((int)cfg.schedSource == r) p += F(" selected");
    p += F(">"); schedRegionAppend(p, r); p += F("</option>");
  }
  p += F("</select></div>");

  p += F("<div class='field'><label>Ваша черга / група</label>"
         "<select name='schedPick' id='schedPick' data-cur='");
  if (cfg.schedSource == 0) {
    p += String((int)cfg.schedGroup);
  } else {
    const char* q = cfg.schedQueue;
    if (strncmp(q, "GPV", 3) == 0) q += 3;
    p += htmlEscape(String(q));
  }
  p += F("'></select></div>"
         "<div class='hint'>Список черг підставляється за вибраною областю: Чернівецька — групи 1–12, "
         "Київ — 1.1–60.1, решта — 1.1–6.2. Свою чергу можна уточнити на сайті оператора. "
         "Дані оновлюються раз на 15 хв; якщо джерело недоступне, діє налаштування fail-open.</div>"
         "<script>(function(){"
         "var s=document.getElementById('schedPick');"
         "var r=document.querySelector(\"select[name='schedSource']\");"
         "if(!s||!r)return;"
         "var cur=s.getAttribute('data-cur')||'',store={},last=null;"
         "function build(){"
         "if(last!==null)store[last]=s.value;"
         "last=r.value;"
         "var src=parseInt(r.value,10),list=[],i;"
         "if(src===0){for(i=1;i<=12;i++)list.push(String(i));}"
         "else if(src===1){for(i=1;i<=60;i++)list.push(i+'.1');}"
         "else{for(i=1;i<=6;i++){list.push(i+'.1');list.push(i+'.2');}}"
         "s.innerHTML='';"
         "for(i=0;i<list.length;i++){var o=document.createElement('option');"
         "o.value=list[i];o.textContent=list[i];s.appendChild(o);}"
         "var want=store.hasOwnProperty(src)?store[src]:cur;"
         "s.value=want;if(s.value!==want)s.value=list[0];"
         "store[src]=s.value;}"
         "r.addEventListener('change',build);build();"
         "})();</script></div>");
  out();

  // ---- повітряні тривоги (NEPTUN) ----
  p += F("<div class='card'><h3>Повітряна тривога</h3>"
         "<div class='field toggle'><label>Повідомляти про тривогу</label>"
         "<input type='checkbox' name='alertEnabled' value='1'");
  if (cfg.alertEnabled) p += F(" checked");
  p += F("></div>");

  p += F("<div class='field'><label>Ваш район</label><select name='alertRaion'>");
  for (int i = 0; i < RAION_COUNT; i++) {
    p += F("<option value='");
    p += String(i + 1);
    p += F("'");
    if ((int)cfg.alertRaion == (i + 1)) p += F(" selected");
    p += F(">");
    raionAppend(p, i);
    p += F("</option>");
    if (p.length() > 900) out();
  }
  p += F("</select></div>");

  p += F("<div class='field'><label>Перевіряти кожних</label><select name='alertPollSec'>");
  const int POLL_OPTS[6] = {15, 30, 60, 120, 300, 600};
  const char* POLL_NAMES[6] = {"15 секунд", "30 секунд", "1 хвилину", "2 хвилини", "5 хвилин", "10 хвилин"};
  bool pollListed = false;
  for (int i = 0; i < 6; i++) if ((int)cfg.alertPollSec == POLL_OPTS[i]) pollListed = true;
  if (!pollListed) {
    p += F("<option value='");
    p += String(cfg.alertPollSec);
    p += F("' selected>");
    p += String(cfg.alertPollSec);
    p += F(" с</option>");
  }
  for (int i = 0; i < 6; i++) {
    p += F("<option value='");
    p += String(POLL_OPTS[i]);
    p += F("'");
    if ((int)cfg.alertPollSec == POLL_OPTS[i]) p += F(" selected");
    p += F(">");
    p += POLL_NAMES[i];
    p += F("</option>");
  }
  p += F("</select></div>");

  p += F("<div class='field toggle'><label>Сповіщати «відбій» зеленим</label>"
         "<input type='checkbox' name='alertNotifyClear' value='1'");
  if (cfg.alertNotifyClear) p += F(" checked");
  p += F("></div>");

  p += F("<div class='hint'>На тривогу — блимне жовтим або червоним і завібрує. "
         "На відбій — зеленим.</div></div>");
  out();

  // ---- додатково (сервіс) ----
  p += F("<details class='card'><summary>Додатково (для сервісу)</summary>"
         "<div class='hint'>Змінюйте лише якщо розумієте, що робите.</div>");
  addNumField(p, "Яскравість при світлі", "minBright", cfg.minBright, 1, 255);
  addNumField(p, "Поріг «день» (0–1023)", "dayLevel", cfg.dayLevel, 0, 1023);
  addNumField(p, "Затухання, мс (0 = різко)", "waveMs", cfg.waveMs, 0, 10000);
  addNumField(p, "Вібрація, мс (0 = вимк.)", "vibMs", cfg.vibMs, 0, 2000);
  addNumField(p, "Прогрів PIR, с", "warmupSec", cfg.warmupSec, 0, 600);
  addNumField(p, "Чутливість PIR (1–50)", "windowMin", cfg.windowMin, 1, WINDOW_SAMPLES);
  addNumField(p, "Захист від залипання PIR, с (0 = вимк.)", "pirStuckSec", cfg.pirStuckSec, 0, 600);
  addNumField(p, "Пауза PIR після вимкнення лампи, с", "pirCoolSec", cfg.pirCoolSec, 0, 300);
  addNumField(p, "Макс. серія HIGH = рух, с (0 = без фільтра)", "pirMaxRunSec", cfg.pirMaxRunSec, 0, 60);
  p += F("<div class='hint'>Типовий профіль: прогрів 60 с, чутливість 42/50, "
         "залипання 60 с, пауза після вимкнення 25 с, вібрація 0. "
         "Обмеження серії (10 с) вмикайте лише якщо перемичка датчика в режимі «L»: "
         "у режимі «H» безперервний рух дає довгу серію, і лампа його проігнорує.</div>");
  out();

  p += F("<div class='row'><label>«МЗ» (можливо заживлені) теж вважати відключенням</label>"
         "<input type='checkbox' name='schedSoft' value='1'");
  if (cfg.schedSoft) p += F(" checked");
  p += F("></div><div class='row'><label>Якщо графік недоступний — дозволяти світло</label>"
         "<input type='checkbox' name='schedFailOpen' value='1'");
  if (cfg.schedFailOpen) p += F(" checked");
  p += F("></div>");

  p += F("<div class='row'><label>Полярність LDR</label><select name='ldrDarkHigh'>");
  p += F("<option value='1'");
  if (cfg.ldrDarkHigh) p += F(" selected");
  p += F(">затемнення збільшує</option><option value='0'");
  if (!cfg.ldrDarkHigh) p += F(" selected");
  p += F(">затемнення зменшує</option></select></div>");
  p += F("<div class='hint'><a href='/?wiz=1'>Запустити майстер налаштування знову</a></div></details>");

  p += F("<button type='submit'>Зберегти</button></form>");
  out();

  // ---- стан графіка відключень ----
  if (cfg.schedEnabled) {
    p += F("<div class='card'><h3>Графік відключень</h3>");

    if (schedTimeOk()) {
      time_t t = time(nullptr);
      struct tm lt;
      localtime_r(&t, &lt);
      char nowbuf[24];
      sprintf(nowbuf, "%02d.%02d.%04d %02d:%02d",
              lt.tm_mday, lt.tm_mon + 1, lt.tm_year + 1900, lt.tm_hour, lt.tm_min);
      p += F("Час пристрою: <b>");
      p += nowbuf;
      p += F("</b><br>");
    } else {
      p += F("Час пристрою: <b>не синхронізовано</b><br>");
    }

    if (!schedValid) {
      p += F("Графік ще не завантажено");
      if (schedErr) {
        p += F(" (причина: ");
        p += schedErrText();
        p += F(")");
      }
      p += F(".");
    } else {
      char sbuf[8];
      schedTimeStr(schedSyncEpoch, sbuf);

      p += F("Область: <b>");
      schedRegionAppend(p, cfg.schedSource);
      p += F("</b><br>");
      p += cfg.schedSource ? F("Черга <b>") : F("Група <b>");
      p += cfg.schedSource ? String(cfg.schedQueue) : String(cfg.schedGroup);
      p += F("</b>, графік на <b>");
      p += String(schedDay);
      p += F(".");
      if (schedMon < 10) p += F("0");
      p += String(schedMon);
      p += F(".");
      p += String(schedYear);
      p += F("</b><br>Оновлено: <b>");
      p += sbuf;
      p += F("</b><br>");

      if (schedDateMatches()) {
        p += F("Зараз у графіку: <b>");
        p += schedSlotOut ? "відключення" : "світло є";
        p += F("</b> (слот ");
        p += String(schedSlotNow / 2);
        p += F(":");
        p += ((schedSlotNow & 1) ? "30" : "00");
        p += F(")<br>");
      } else {
        p += F("Графік на іншу дату — дані не застосовуються.<br>");
      }

      p += F("Лампа може світити: <b>");
      p += schedAllowedNow ? "так" : "ні";
      p += F("</b>");

      if (schedErr) {
        p += F("<br>Остання помилка: ");
        p += schedErrText();
      }

      p += F("<div class='schedbar'>");
      for (int i = 0; i < SCHED_SLOTS; i++) {
        bool b = ((schedBits[i >> 3] >> (i & 7)) & 1) != 0;
        p += F("<i class='");
        p += b ? "b" : "g";
        p += F("'></i>");
      }
      p += F("</div><div class='hint'>00:00 → 23:30; червоне = відключення</div>");
    }

    p += F("<form method='POST' action='/schedsync'><button class='ghost' type='submit'>Оновити графік зараз</button></form>");
    p += F("</div>");
    out();
  }

  if (server.hasArg("sched")) {
    p += F("<div class='card' id='upd'>Оновлення графіка запущено… сторінка оновиться автоматично.</div>");
    p += F("<script>setTimeout(function(){location.href='/';},20000);</script>");
  }

  // ---- стан повітряних тривог ----
  if (cfg.alertEnabled) {
    p += F("<div class='card' style='border:1px solid #6f5a2f'><h3>Повітряна тривога</h3>");

    p += F("Район: <b>");
    raionAppend(p, cfg.alertRaion - 1);
    p += F("</b><br>");

    p += F("Стан: <b>");
    p += alertLevelName(cfg.alertLastLevel);
    p += F("</b>");
    if (cfg.alertLastLevel > 0 && alertSince[0]) {
      p += F(" — з <b>");
      p += alertSince;
      p += F(" UTC</b>");
    }
    p += F("<br>");

    if (alertCheckAt) {
      p += F("Перевірено <b>");
      p += String((millis() - alertCheckAt) / 1000UL);
      p += F(" с тому</b>");
      if (alertErr) { p += F(" · "); p += alertErrText(); }
      p += F("<br>");
    } else {
      p += F("Ще не перевірено<br>");
    }

    p += F("Період перевірки: <b>");
    p += String(cfg.alertPollSec);
    p += F(" с<br>");

    p += F("<form method='POST' action='/alertsync'><button class='ghost' type='submit'>Перевірити зараз</button></form>");
    p += F("</div>");
    out();
  }

  if (server.hasArg("alert")) {
    p += F("<div class='card' id='upd'>Перевірку тривоги запущено… сторінка оновиться автоматично.</div>");
    p += F("<script>setTimeout(function(){location.href='/';},15000);</script>");
  }

  p += F(R"HTML(<div class='card'><h3>Скидання</h3>
<form method='POST' action='/reset'><button class='ghost' type='submit'>Скинути налаштування до заводських</button></form>
<form method='POST' action='/reboot'><button class='danger' type='submit'>Перезавантажити пристрій</button></form>
</div>)HTML");
  out();

  // ---- Wi-Fi ----
  p += F("<div class='card'><h3>Wi-Fi</h3>");
  if (WiFi.status() == WL_CONNECTED) {
    p += "Підключено до <b>" + htmlEscape(WiFi.SSID()) + "</b><br>IP: <b>" + WiFi.localIP().toString() + "</b>";
    if (mdnsRunning) {
      p += F("<br><a href='http://");
      p += mdnsHost;
      p += F(".local/'>http://");
      p += mdnsHost;
      p += F(".local/</a>");
    }
  } else {
    p += F("Пристрій не підключений до роутера");
  }
  p += F("<br><br><a href='/?rescan=1'>Оновити список мереж</a> &nbsp;|&nbsp; <a href='/forget'>Забути мережу</a></div>");
  out();

  p += F("<form method='POST' action='/connect'><div class='card'><h3>Змінити мережу</h3>");

  if (netCount == 0) {
    p += F("<div class='net'>Нічого не знайдено — спробуйте оновити</div>");
  }
  for (int i = 0; i < netCount; i++) {
    p += F("<label class='net'><input type='radio' name='ssid' value='");
    p += htmlEscape(nets[i].ssid);
    p += F("' required><span>");
    p += htmlEscape(nets[i].ssid);
    p += F("</span><small>");
    if (nets[i].enc) p += "🔒 ";
    p += String(nets[i].rssi);
    p += F(" dBm</small></label>");
    if (p.length() > 700) out();   // список мереж теж віддаємо шматками
  }
  out();

  p += F(R"HTML(</div>
<input type='password' name='pass' placeholder='пароль (порожній для відкритої мережі)'>
<button type='submit'>Підключити</button></form>)HTML");
  out();

  p += F(R"HTML(<script>
const $=id=>document.getElementById(id);
async function tick(){
  try{
    const j=await (await fetch('/status',{cache:'no-store'})).json();
    $('s_state').textContent=j.lamp;
    $('s_lvl').textContent=j.lvl;
    $('s_ldr').textContent=j.ldr;
    $('s_br').textContent=j.br;
    $('s_pir').textContent=(j.pir?'рух':'немає')+' ('+j.hc+'/50)';
    $('s_day').textContent=j.day?'день':'ніч';
    $('s_wifi').textContent=j.ssid?(j.ssid+' · '+j.ip):'немає';
    $('s_ap').textContent=j.ap?'увімкнена':'вимкнена';
    $('s_up').textContent=j.up+' с';
    $('s_sched').textContent = j.sch
      ? ((j.sv?(j.so?'відключення':'світло є'):'немає даних')+(j.su?' · дозволено':' · заборонено'))
      : 'вимкнено';
    if(j.al!==undefined){
      $('s_alert').textContent=(j.al==2?'червоний':j.al==1?'жовтий':'немає')+(j.an?' (блимає)':'');
    }
  }catch(e){}
}
tick();setInterval(tick,2000);
</script></body></html>)HTML");
  out();
  httpChunkEnd();
}

// ---------- збереження / скидання / перезавантаження ----------
int argInt(const char* name, int defVal) {
  if (!server.hasArg(String(name))) return defVal;
  String v = server.arg(String(name));
  v.trim();
  if (v.length() == 0) return defVal;
  return v.toInt();
}

// ---------- майстер першого запуску (для покупця) ----------
// Кожен крок — окрема форма. Чекбокси /save рахує за фактом наявності поля,
// тому стан, який не показуємо на кроці, передаємо прихованими полями.
static void wizHiddenFlags(String& p) {
  if (cfg.pirEnabled)       p += F("<input type='hidden' name='pirEnabled' value='1'>");
  if (cfg.schedEnabled)     p += F("<input type='hidden' name='schedEnabled' value='1'>");
  if (cfg.schedSoft)        p += F("<input type='hidden' name='schedSoft' value='1'>");
  if (cfg.schedFailOpen)    p += F("<input type='hidden' name='schedFailOpen' value='1'>");
  if (cfg.alertNotifyClear) p += F("<input type='hidden' name='alertNotifyClear' value='1'>");
}

void pageWizard() {
  int step = argInt("step", 1);
  if (step < 1) step = 1;
  if (step > 3) step = 3;

  String p;
  p.reserve(1200);
  httpChunkHeader();
  auto out = [&]() {
    httpChunk(p);
    p = "";
  };

  p += F(R"HTML(<!DOCTYPE html><html lang='uk'><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Нічник — налаштування</title><style>
body{margin:0;padding:16px;font-family:system-ui,Arial,sans-serif;background:#0f141a;color:#e8eef5;color-scheme:dark}
h2{margin:0 0 14px;font-size:20px}h3{margin:0 0 12px;font-size:15px;color:#9fb3c8}
.card{background:#1a222b;border-radius:12px;padding:14px;margin-bottom:14px}
.field{margin:14px 0}.field label{display:block;font-size:13px;color:#9fb3c8;margin-bottom:6px}
.field select{width:100%;padding:12px;border-radius:10px;border:1px solid #33414f;background:#0d1218;color:#fff;font-size:16px;box-sizing:border-box}
input[type=range]{width:100%;accent-color:#2f81f7;height:28px}
input[type=password]{width:100%;padding:12px;border-radius:10px;border:1px solid #33414f;background:#0d1218;color:#fff;font-size:16px;box-sizing:border-box}
input[type=checkbox]{width:24px;height:24px;accent-color:#2f81f7}
button{width:100%;padding:14px;margin-top:12px;border:0;border-radius:10px;background:#2f81f7;color:#fff;font-size:16px;font-weight:600}
a{color:#79b8ff}.hint{font-size:12px;color:#8fa3b8;margin-top:8px}
.steps{display:flex;gap:6px;margin:0 0 14px}.steps i{flex:1;height:5px;border-radius:3px;background:#2a3542}
.steps i.on{background:#2f81f7}
.net{display:flex;align-items:center;gap:10px;padding:10px;border-radius:8px;margin:4px 0;background:#212c38}
.net small{margin-left:auto;color:#8fa3b8;white-space:nowrap}
.big{font-size:16px;font-weight:700}
</style></head><body><h2>Нічник</h2><div class='steps'>)HTML");
  for (int i = 1; i <= 3; i++) {
    if (i <= step) p += F("<i class='on'></i>");
    else           p += F("<i></i>");
  }
  p += F("</div>");
  out();

  if (step == 1) {
    p += F("<div class='card'><h3>Крок 1 з 3 — Wi-Fi</h3>");
    if (WiFi.status() == WL_CONNECTED) {
      p += F("<div class='big'>✅ ");
      p += htmlEscape(WiFi.SSID());
      p += F("</div><div class='hint'>Підключено, IP ");
      p += WiFi.localIP().toString();
      p += F("</div><a href='/?wiz=1&step=2'><button type='button'>Далі</button></a>");
    } else {
      p += F("<div class='hint' style='margin-bottom:10px'>Виберіть домашню мережу 2.4 ГГц "
             "(мережі 5 ГГц не підходять).</div><form method='POST' action='/connect'>");
      if (netCount == 0) {
        p += F("<div class='net'>Мереж не знайдено — <a href='/?rescan=1'>оновити список</a></div>");
      }
      for (int i = 0; i < netCount; i++) {
        p += F("<label class='net'><input type='radio' name='ssid' value='");
        p += htmlEscape(nets[i].ssid);
        p += F("' required><span>");
        p += htmlEscape(nets[i].ssid);
        p += F("</span><small>");
        if (nets[i].enc) p += "🔒 ";
        p += String(nets[i].rssi);
        p += F("</small></label>");
        if (p.length() > 900) out();
      }
      p += F("<input type='password' name='pass' placeholder='пароль від Wi-Fi'>"
             "<button type='submit'>Підключити</button></form>"
             "<div class='hint'>Після підключення відкрийте цю сторінку знову — "
             "адреса буде вже в мережі дому.</div>");
    }
    p += F("<div class='hint'><a href='/?wiz=1&step=2'>Пропустити цей крок →</a></div></div>");
  }

  if (step == 2) {
    p += F("<div class='card'><h3>Крок 2 з 3 — повітряна тривога</h3>"
           "<form method='POST' action='/save'>");
    wizHiddenFlags(p);
    p += F("<input type='hidden' name='next' value='/?wiz=1&step=3'>"
           "<div class='field'><label>Повідомляти про тривогу</label>"
           "<input type='checkbox' name='alertEnabled' value='1' checked></div>"
           "<div class='hint'>На тривогу нічник блимне жовтим або червоним і завібрує, "
           "на відбій — зеленим.</div>"
           "<div class='field'><label>Ваш район</label><select name='alertRaion'>");
    for (int i = 0; i < RAION_COUNT; i++) {
      p += F("<option value='");
      p += String(i + 1);
      p += F("'");
      if ((int)cfg.alertRaion == (i + 1)) p += F(" selected");
      p += F(">");
      raionAppend(p, i);
      p += F("</option>");
      if (p.length() > 900) out();
    }
    p += F("</select></div><button type='submit'>Далі</button></form>"
           "<div class='hint'><a href='/?wiz=1&step=1'>← Назад</a></div></div>");
  }

  if (step == 3) {
    p += F("<div class='card'><h3>Крок 3 з 3 — як світити</h3>"
           "<form method='POST' action='/save'>");
    wizHiddenFlags(p);
    if (cfg.alertEnabled) p += F("<input type='hidden' name='alertEnabled' value='1'>");
    p += F("<input type='hidden' name='next' value='/'>"
           "<input type='hidden' name='setupDone' value='1'>"
           "<div class='field'><label>Коли вмикати світло</label><select name='pirMode'>"
           "<option value='motion' selected>Коли хтось рухається (датчик руху)</option>"
           "<option value='always'>Світити всю ніч, щойно стемніє</option></select></div>"
           "<div class='field'><label>Яскравість у темряві</label>"
           "<input type='range' name='maxBright' min='20' max='255' step='5' value='");
    p += String(cfg.maxBright);
    p += F("'></div><button type='submit'>Готово</button></form>"
           "<div class='hint'><a href='/?wiz=1&step=2'>← Назад</a></div></div>");
  }

  p += F("</body></html>");
  out();
  httpChunkEnd();
}

void handleSave() {
  uint8_t oldGroup = cfg.schedGroup;
  uint8_t oldSource = cfg.schedSource;
  uint8_t oldAlertEnabled = cfg.alertEnabled;
  uint16_t oldRaion = cfg.alertRaion;

  cfg.minBright   = argInt("minBright", cfg.minBright);
  cfg.maxBright   = argInt("maxBright", cfg.maxBright);
  cfg.dayLevel    = argInt("dayLevel", cfg.dayLevel);
  cfg.nightArm    = argInt("nightArm", cfg.nightArm);
  cfg.holdSec     = argInt("holdSec", cfg.holdSec);
  cfg.waveMs      = argInt("waveMs", cfg.waveMs);
  cfg.vibMs       = argInt("vibMs", cfg.vibMs);
  cfg.warmupSec   = argInt("warmupSec", cfg.warmupSec);
  cfg.windowMin   = argInt("windowMin", cfg.windowMin);
  cfg.pirStuckSec = argInt("pirStuckSec", cfg.pirStuckSec);
  cfg.pirCoolSec  = argInt("pirCoolSec", cfg.pirCoolSec);
  cfg.pirMaxRunSec = argInt("pirMaxRunSec", cfg.pirMaxRunSec);
  if (server.hasArg("pirMode")) {
    // людський перемикач: motion = реагувати на рух, always = світити всю ніч
    cfg.pirEnabled = (server.arg("pirMode") == "always") ? 0 : 1;
  } else {
    cfg.pirEnabled = server.hasArg("pirEnabled") ? 1 : 0;
  }
  cfg.pirOffOnly  = server.hasArg("pirOffOnly") ? 1 : 0;
  cfg.ldrDarkHigh = argInt("ldrDarkHigh", cfg.ldrDarkHigh) ? 1 : 0;

  cfg.schedEnabled  = server.hasArg("schedEnabled")  ? 1 : 0;
  cfg.schedGroup    = argInt("schedGroup", cfg.schedGroup);
  if (server.hasArg("schedSource")) cfg.schedSource = argInt("schedSource", cfg.schedSource);
  if (server.hasArg("schedPick")) {
    // один список для всіх: Чернівецька -> група 1..12, інші -> черга 1.1..6.2 (Київ 1.1..60.1)
    String v = server.arg("schedPick"); v.trim();
    if (v.startsWith("GPV")) v = v.substring(3);
    if (cfg.schedSource == 0) {
      int g = v.toInt();
      if (g >= 1 && g <= SCHED_GROUPS) cfg.schedGroup = (uint8_t)g;
    } else if (v.length() && v.length() < sizeof(cfg.schedQueue)) {
      v.toCharArray(cfg.schedQueue, sizeof(cfg.schedQueue));
    }
  } else if (server.hasArg("schedQueue")) {
    String v = server.arg("schedQueue"); v.trim();
    v.substring(0, sizeof(cfg.schedQueue) - 1).toCharArray(cfg.schedQueue, sizeof(cfg.schedQueue));
  }
  cfg.schedSoft     = server.hasArg("schedSoft")     ? 1 : 0;
  cfg.schedFailOpen = server.hasArg("schedFailOpen") ? 1 : 0;

  cfg.alertEnabled     = server.hasArg("alertEnabled")     ? 1 : 0;
  cfg.alertNotifyClear = server.hasArg("alertNotifyClear") ? 1 : 0;
  cfg.alertRaion    = argInt("alertRaion", cfg.alertRaion);
  cfg.alertPollSec  = argInt("alertPollSec", cfg.alertPollSec);

  // новий район / вимкнення -> стан невідомий, знову «не блимали»
  if (cfg.alertRaion != oldRaion || !cfg.alertEnabled) cfg.alertLastLevel = 0;
  if (cfg.alertRaion != oldRaion || cfg.alertKey[0] == 0)
    raionKey(cfg.alertRaion - 1, cfg.alertKey, sizeof(cfg.alertKey));   // ключ у EEPROM

  cfg.setupDone = server.hasArg("setupDone") ? 1 : cfg.setupDone;

  settingsSave();
  targetBrightness = brightnessForLdr(ldrSmooth);   // одразу застосувати
  if (cfg.schedGroup != oldGroup || cfg.schedSource != oldSource || server.hasArg("schedSource"))
    schedValid = false;                                  // кеш іншого джерела/черги не годиться
  if (cfg.schedEnabled) schedForce = true;              // підтягнути графік одразу
  schedUpdate();                                        // і графік
  if (cfg.alertEnabled) alertForce = true;              // і тривогу підтягнути
  Serial.printf("SAVE: sch=%u src=%u grp=%u q=%s soft=%u fo=%u | al=%u ra=%u ps=%u nc=%u | pir=%u\n",
                cfg.schedEnabled, cfg.schedSource, cfg.schedGroup, cfg.schedQueue, cfg.schedSoft, cfg.schedFailOpen,
                cfg.alertEnabled, cfg.alertRaion, cfg.alertPollSec, cfg.alertNotifyClear,
                cfg.pirEnabled);

  server.sendHeader("Location", server.hasArg("next") ? server.arg("next") : String("/?saved=1"), true);
  server.send(302, "text/plain", "");
}

void handleReset() {
  settingsDefaults();
  settingsSave();
  targetBrightness = brightnessForLdr(ldrSmooth);
  schedUpdate();

  server.sendHeader("Location", "/?saved=1", true);
  server.send(302, "text/plain", "");
}

void handleSchedSync() {
  schedForce = true;                    // реальний запит почнеться після відповіді
  server.sendHeader("Location", "/?sched=1", true);
  server.send(302, "text/plain", "");
}

// тестовий хук: /beztest?src=7&q=1.1 — примусово перевірити дублер bezsvitla
void handleBezTest() {
  bezTestPending = true;
  bezTestSrc = (uint8_t)argInt("src", cfg.schedSource);
  String q = server.arg("q");
  if (!q.length()) q = cfg.schedQueue;
  q.toCharArray(bezTestQ, sizeof(bezTestQ));
  server.send(200, "text/plain", "ok, результат у serial");
}

void handleAlertSync() {
  alertForce = true;                    // реальний запит почнеться після відповіді
  server.sendHeader("Location", "/?alert=1", true);
  server.send(302, "text/plain", "");
}

// тестовий хук: http://<ip>/alerttest?lvl=2 (червоний), ?lvl=1 (жовтий), ?lvl=0 (відбій)
void handleAlertTest() {
  int lvl = argInt("lvl", 0);
  if (lvl < 0) lvl = 0;
  if (lvl > 2) lvl = 2;
  alertValid = true;
  alertLevelNow = (uint8_t)lvl;
  alertCheckAt = millis();
  if (cfg.alertEnabled) alertApply((uint8_t)lvl);
  server.send(302, "text/plain", "");
}

// ДАГОСЛИТ: вимкнути радіо на N с і порахувати сплески PIR
// (/wifitest?off=180) — якщо за radio-off сплесків немає, а з радіо є,
// причина просадок — Wi-Fi передачі (до ~290 мА імпульсами).
void handleWifiTest() {
  long off = argInt("off", 120);
  if (off < 0) off = 0;
  if (off > 600) off = 600;
  if (off == 0) {
    wifiOffApplyAt = 0;
    wifiOffUntil = 0;
    server.send(200, "text/plain", "radio restore armed");
    Serial.println("WIFITEST: restore armed");
    return;
  }
  wifiOffApplyAt = millis() + 1500UL;
  wifiOffUntil = millis() + 1500UL + (unsigned long)off * 1000UL;
  Serial.printf("WIFITEST: radio OFF на %ld s (подій PIR зараз %u)\n", off, pirEvents);
  server.send(200, "text/plain", String("radio off ") + String(off) + " s, events=" + String(pirEvents));
}

void handleReboot() {
  server.send(200, "text/html",
    F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<meta http-equiv='refresh' content='8;url=/'>"
      "<body style='font-family:system-ui;background:#0f141a;color:#eee;padding:16px'>"
      "<h2>Перезавантаження…</h2><p>Пристрій перезапускається. Сторінка оновиться сама.</p></body></html>"));
  delay(400);
  ESP.restart();
}

void handleConnect() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

  if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 63 ||
      (pass.length() > 0 && pass.length() < 8)) {
    server.send(200, "text/html",
      F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<body style='font-family:system-ui;background:#0f141a;color:#eee;padding:16px'>"
        "<h2>Перевірте мережу й пароль</h2><p>Виберіть мережу. Пароль: 8–63 символи або порожній для відкритої мережі.</p><a style='color:#79b8ff' href='/'>← Назад</a></body></html>"));
    return;
  }

  saveCreds(ssid, pass);
  apJoinPending = requestFromAP();
  if (apJoinPending) {
    // Let iOS receive the POST response BEFORE a possible AP channel change.
    apJoinApplyAt = millis() + 1200UL;
  } else {
    WiFi.setAutoReconnect(true);
    WiFi.disconnect(false);
    WiFi.begin(ssid.c_str(), pass.c_str());
  }
  connectPending = true;
  connectStartedAt = millis();
  staRetryAt = connectStartedAt + WIFI_TIMEOUT;

  String p;
  p.reserve(1400);
  p += F(R"HTML(<!DOCTYPE html><html lang='uk'><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Підключення…</title><style>
body{margin:0;padding:16px;font-family:system-ui,Arial,sans-serif;background:#0f141a;color:#e8eef5;color-scheme:dark}
h2{font-size:20px}.card{background:#1a222b;border-radius:12px;padding:16px;margin-bottom:14px}
a{color:#79b8ff}</style></head><body><h2>Підключення…</h2>
<div class='card' id='st'>Пробую під'єднатись до <b>)HTML");
  p += htmlEscape(ssid);
  p += F(R"HTML(</b>…</div>
<script>
async function poll(){
  try{
    const r = await fetch('/status',{cache:'no-store'});
    const j = await r.json();
    const el = document.getElementById('st');
    if(j.state==='ok'){
      el.replaceChildren(document.createTextNode('✅ Підключено до '+j.ssid+'. Поверніть телефон до домашньої мережі та відкрийте: '));
      const link=document.createElement('a');link.href='http://'+j.ip+'/';link.textContent=link.href;el.appendChild(link);
      document.title = 'Готово';
      return;
    }
    if(j.state==='fail'){
      el.innerHTML = '❌ Не вдалося підключитися.<br>Перевірте пароль і спробуйте ще.<br><br><a href="/">← Назад</a>';
      return;
    }
  }catch(e){}
  setTimeout(poll,1000);
}
poll();
</script></body></html>)HTML");

  server.send(200, "text/html", p);
}

void handleStatus() {
  String state;
  if (WiFi.status() == WL_CONNECTED) {
    state = "ok";
    connectPending = false;
  } else if (connectPending && (millis() - connectStartedAt > WIFI_TIMEOUT)) {
    state = "fail";
  } else if (connectPending) {
    state = "connecting";
  } else {
    state = "idle";
  }

  String j = "{\"state\":\"" + state + "\"";
  if (requestFromAP()) {
    j += ",\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", j);
    return;
  }
  j += ",\"lamp\":\"" + String(stateName(lampState)) + "\"";
  j += ",\"ldr\":" + String(ldrRaw);
  j += ",\"lvl\":" + String(lightLevelFor(ldrSmooth));
  j += ",\"br\":" + String(currentBrightness);
  j += ",\"pir\":" + String(digitalRead(PIN_PIR));
  j += ",\"hc\":" + String(highCount);
  j += ",\"mot\":" + String(motionNow ? 1 : 0);
  j += ",\"stuck\":" + String(pirStuck ? 1 : 0);
  j += ",\"pe\":" + String(cfg.pirEnabled ? 1 : 0);
  j += ",\"pss\":" + String(cfg.pirStuckSec);
  j += ",\"po\":" + String(cfg.pirOffOnly ? 1 : 0);
  j += ",\"pc\":" + String(cfg.pirCoolSec);
  j += ",\"pm\":" + String(cfg.pirMaxRunSec);
  j += ",\"gate\":" + String(pirGated(millis()) ? 1 : 0);
  j += ",\"ev\":" + String(pirEvents);
  j += ",\"rn\":" + String(radioNoiseRuns);
  j += ",\"hi\":" + String(pirLastHighMs);
  j += ",\"hn\":" + String(pirHighNowMs);
  j += ",\"high\":" + String(highCount);
  j += ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"day\":" + String(daylight ? 1 : 0);
  j += ",\"ap\":" + String(apRunning ? 1 : 0);
  j += ",\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\"";
  j += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  j += ",\"up\":" + String((millis() - bootAt) / 1000UL);
  j += ",\"heap\":" + String(ESP.getFreeHeap());
  if (cfg.schedEnabled) {
    j += ",\"sch\":1";
    j += ",\"grp\":" + String(cfg.schedGroup);
    j += ",\"su\":" + String(schedAllowedNow ? 1 : 0);
    j += ",\"sv\":" + String(schedValid ? 1 : 0);
    j += ",\"so\":" + String(schedSlotOut ? 1 : 0);
  }
  if (cfg.alertEnabled) {
    j += ",\"al\":" + String(cfg.alertLastLevel);
    j += ",\"ra\":" + String(cfg.alertRaion);
    j += ",\"an\":" + String(alertNotify);
  }
  j += "}";
  server.send(200, "application/json", j);
}

// живий діагностичний екран (оновлюється щосекунди): /live
// допомагає очима перевірити датчики — LDR (закрий/відкрий) і PIR (махни рукою)
void handleLive() {
  bool warmup = (millis() - bootAt) < warmupMs;
  int lvl = lightLevelFor(ldrSmooth);
  String p = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
               "<meta name='viewport' content='width=device-width,initial-scale=1'>"
               "<meta http-equiv='refresh' content='1'><title>live</title>"
               "<style>body{font:15px/1.7 monospace;padding:14px;background:#141414;color:#ddd}"
               ".v{color:#7f7;font-weight:bold}.w{color:#fa4}.s{color:#777}"
               "hr{border:0;border-top:1px solid #333}</style></head><body>");
  p += F("<div class='s'>оновлення щосекунди &mdash; /live</div><hr>");
  p += F("<div>LDR raw: <span class='v'>"); p += String(ldrRaw);
  p += F("</span> &rarr; LVL <span class='v'>"); p += String(lvl);
  p += F("</span></div><div>Режим: <span class='v'>");
  p += daylight ? F("ДЕНЬ (лампа не світить)") : F("НІЧ");
  p += F("</span></div><div>Пороги: день &ge; "); p += String(cfg.dayLevel);
  p += F(" | ніч &lt; "); p += String(cfg.nightArm);
  p += F("</div><div>PIR — джерело вмикання: <span class='v'>");
  p += cfg.pirEnabled ? F("увімкнено") : F("ВИМКНЕНО (світить за графіком)");
  p += F("</span></div><hr><div>PIR (пін): <span class='v'>"); p += String(digitalRead(PIN_PIR));
  p += F("</span> &nbsp; подій: <span class='v'>"); p += String(pirEvents);
  p += F("</span> &nbsp; наводок Wi-Fi: <span class='w'>"); p += String(radioNoiseRuns);
  p += F("</span> &nbsp; останя HIGH: <span class='v'>"); p += String(pirLastHighMs);
  p += F(" мс");
  p += F(" &nbsp; HIGH у вікні: <span class='v'>"); p += String(highCount);
  p += F("/"); p += String(WINDOW_SAMPLES);
  p += F("</span> (поріг "); p += String(cfg.windowMin);
  p += F(")</div><div>Рух: <span class='v'>"); p += motionNow ? F("є") : F("нема");
  p += F("</span> &nbsp; STUCK: <span class='w'>"); p += pirStuck ? F("1") : F("0");
  p += F("</span> &nbsp; WARMUP: "); p += warmup ? F("1") : F("0");
  p += F(" &nbsp; PIR-пауза: <span class='w'>");
  long bleft = (long)(pirBlankUntil - millis());
  p += bleft > 0 ? F("так") : F("ні");
  p += F(" ("); p += String(bleft > 0 ? bleft / 1000 : 0); p += F(" с)");  p += F("</span> &nbsp; чекає LOW: <span class='w'>");
  p += pirGated(millis()) ? F("так") : F("ні");
  if (pirWaitLow && pirLowTracked) {
    unsigned long lowFor = millis() - pirLowSince;
    p += F(" ("); p += String(lowFor >= PIR_LOW_STABLE_MS ? 0 : PIR_LOW_STABLE_MS - lowFor);
    p += F(" мс до готовності)");
  }
  p += F("</span>");
  p += F("</span> &nbsp; рух у ON / WAVE: <span class='w'>рахуємо / рахуємо");
  p += F("</span> &nbsp; макс. серія: <span class='w'>");
  if (cfg.pirMaxRunSec > 0) { p += String(cfg.pirMaxRunSec); p += F(" с"); }
  else p += F("вимк.");
  p += F("</span>");
  p += F("</div><hr><div>Стан лампи: <span class='v'>"); p += stateName(lampState);
  p += F("</span> &nbsp; яскравість: "); p += String(currentBrightness);
  p += F("</div>");
  p += F("<div>Остання зміна: <span class='v'>"); p += lampWhy;
  p += F("</span> &nbsp; hold: "); p += String(cfg.holdSec);
  p += F(" с &nbsp; wave: "); p += String(cfg.waveMs);
  p += F(" мс &nbsp; warmup: "); p += String(warmupMs / 1000UL);
  p += F(" с &nbsp; cool: "); p += String(cfg.pirCoolSec);
  p += F(" с &nbsp; поріг: "); p += String(cfg.windowMin); p += F("/50");
  p += F("</div><div>Reset: <span class='w'>"); p += resetReason;
  p += F("</span> &nbsp; boots: <span class='w'>"); p += String(bootCount);
  p += F("</span> &nbsp; змін стану: "); p += String(lampChanges);
  p += F("</div>");
  if (pirStuck) {
    p += F("<div style='color:#f66;font-weight:bold'>УВАГА: PIR тримає HIGH &gt; ");
    p += String(cfg.pirStuckSec);
    p += F(" с. Перевір живлення/GND/OUT (D5) датчика руху, чутливість і "
           "чи в полі зору немає теплого (кільце, регулятор, повітря).</div>");
  }
  if (ldrRaw >= 1015)
    p += F("<div style='color:#fa4'>LDR на максимумі (темрява або обрив). "
           "Якщо в кімнаті не темно &mdash; перевір A0 / 3.3V / GND подільника.</div>");
  p += F("<div>Wi-Fi: "); p += WiFi.status() == WL_CONNECTED ? F("ok") : F("нема");
  p += F(" &nbsp; RSSI: "); p += String(WiFi.RSSI());
  p += F("</div><div class='s'>heap "); p += String(ESP.getFreeHeap());
  p += F(" &nbsp; up "); p += String((millis() - bootAt) / 1000UL);
  p += F(" c</div><hr><div class='s'>Закрий/відкрий LDR &mdash; LVL має змінюватись.<br>"
         "Махни рукою перед PIR &mdash; пін PIR і HIGH мають змінюватись.</div></body></html>");
  server.send(200, "text/html", p);
}

// Окрема PIR-діагностика. Лог зберігається тільки в браузері, доки сторінка відкрита.
void handlePirPage() {
  static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html lang='uk'><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>PIR — жива діагностика</title><style>
*{box-sizing:border-box}body{margin:0;padding:14px;font:14px system-ui,Arial,sans-serif;background:#0f141a;color:#e8eef5;color-scheme:dark}
h2{margin:0 0 12px;font-size:20px}.card{background:#1a222b;border-radius:12px;padding:12px;margin-bottom:12px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:7px}.s{background:#212c38;border-radius:8px;padding:8px}
.s b{display:block;font-size:19px;color:#fff}.hi{color:#6dff8a!important}.lo{color:#8fa3b8!important}.warn{color:#ffb45e!important}
canvas{display:block;width:100%;height:260px;background:#0a0e13;border-radius:8px}
.legend{display:flex;gap:16px;margin:7px 0 0;color:#9fb3c8;font-size:12px}.g{color:#5cff78}.o{color:#ffad42}
.buttons{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:9px}button,a.btn{border:0;border-radius:8px;padding:9px 12px;background:#2f81f7;color:#fff;text-decoration:none;font-weight:600}
button.gray,a.gray{background:#2a3542}pre{height:330px;overflow:auto;margin:0;padding:10px;border-radius:8px;background:#090d11;color:#c9d6e3;font:12px/1.45 monospace;white-space:pre}
.note{color:#8fa3b8;font-size:12px;margin-top:8px}.bad{color:#ff6b6b}body.offline #conn{color:#ff6b6b!important}
</style></head><body>
<h2>PIR — жива діагностика</h2>
<div class='card stats'>
 <div class='s'>Пін S<b id='pin'>—</b></div><div class='s'>HIGH у вікні<b id='hc'>—/50</b></div>
 <div class='s'>HIGH триває<b id='hn'>—</b></div><div class='s'>Рух фільтра<b id='mot'>—</b></div>
 <div class='s'>Gate / блок<b id='gate'>—</b></div><div class='s'>Wi-Fi наводка<b id='rf'>—</b></div>
 <div class='s'>Подій / RF<b id='ev'>—</b></div><div class='s'>Лампа<b id='lamp'>—</b></div>
 <div class='s'>З'єднання<b id='conn'>очікування</b></div>
</div>
<div class='card'><canvas id='graph'></canvas><div class='legend'><span class='g'>● пін S: LOW/HIGH</span><span class='o'>● HIGH-семпли: 0–50</span><span>вікно: останні 60 с</span></div></div>
<div class='card'><div class='buttons'><button id='pause'>Пауза</button><button class='gray' id='clear'>Очистити</button><button class='gray' id='save'>Завантажити лог</button><button class='gray' id='events'>Журнал подій</button><a class='btn gray' href='/'>← Налаштування</a></div>
<pre id='log'></pre><div class='note'>Лог і графік існують лише в цьому браузері. Коли вкладка прихована або закрита, опитування припиняється. Часте Wi-Fi-опитування саме може створювати наводки на погано екранований PIR.</div></div>
<script>
const $=id=>document.getElementById(id), cv=$('graph'), cx=cv.getContext('2d');
let points=[], lines=[], paused=false, busy=false, timer=0, last=null;
const lampNames=['OFF','ON','WAVE','ALERT'];
function stamp(){const d=new Date();return d.toLocaleTimeString('uk-UA',{hour12:false})+'.'+String(d.getMilliseconds()).padStart(3,'0')}
function addLog(j){
 const changes=[];
 if(!last||j.pir!==last.pir) changes.push('PIN='+(j.pir?'HIGH':'LOW'));
 if(!last||j.mot!==last.mot) changes.push('MOTION='+j.mot);
 if(!last||j.gate!==last.gate) changes.push('GATE='+j.gate);
 if(!last||j.blk!==last.blk) changes.push('BLOCK='+j.blk);
 if(!last||j.rf!==last.rf) changes.push('RF='+j.rf);
 if(!last||j.stuck!==last.stuck) changes.push('STUCK='+j.stuck);
 if(!last||j.lamp!==last.lamp) changes.push('LAMP->'+lampNames[j.lamp]+'/'+j.why+' hold='+j.hl+'с');
 if(last&&j.bc!==last.bc) changes.push('REBOOT#'+j.bc+' '+(j.rr||''));
 let mark=changes.length?' | '+changes.join(' '):'';
 lines.push(`${stamp()}  S=${j.pir} HIGH=${j.hc}/50 now=${j.hn}ms mot=${j.mot} gate=${j.gate} blk=${j.blk} rf=${j.rf} lamp=${lampNames[j.lamp]||j.lamp}${mark}`);
 const visible=lines.length>1200?lines.slice(-1200):lines;
 $('log').textContent=visible.join('\n'); $('log').scrollTop=$('log').scrollHeight; last=j;
}
function setv(id,text,cls=''){const e=$(id);e.textContent=text;e.className=cls}
function show(j){
 setv('pin',j.pir?'HIGH (1)':'LOW (0)',j.pir?'hi':'lo'); setv('hc',j.hc+'/50',j.hc>=35?'hi':'');
 setv('hn',j.hn+' мс',j.hn>=1000?'hi':''); setv('mot',j.mot?'РУХ':'немає',j.mot?'hi':'lo');
 setv('gate',j.gate+'/'+j.blk,(j.gate||j.blk)?'warn':'lo'); setv('rf',j.rf?'ТАК':'ні',j.rf?'warn':'lo');
 setv('ev',j.ev+' / '+j.rn); setv('lamp',lampNames[j.lamp]||j.lamp); setv('conn','онлайн','hi');
 points.push({t:j.ms,p:j.pir,h:j.hc}); while(points.length&&j.ms-points[0].t>60000)points.shift();
 addLog(j); draw(); document.body.classList.remove('offline');
}
function draw(){
 const dpr=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;if(cv.width!==w*dpr||cv.height!==h*dpr){cv.width=w*dpr;cv.height=h*dpr}cx.setTransform(dpr,0,0,dpr,0,0);cx.clearRect(0,0,w,h);
 cx.strokeStyle='#273544';cx.lineWidth=1;for(let i=0;i<=5;i++){let y=12+(h-30)*i/5;cx.beginPath();cx.moveTo(38,y);cx.lineTo(w-8,y);cx.stroke()}
 cx.fillStyle='#728399';cx.font='11px monospace';cx.fillText('HIGH',3,20);cx.fillText('LOW',9,h-17);if(!points.length)return;
 const newest=points[points.length-1].t,start=newest-60000,x=t=>38+Math.max(0,t-start)/60000*(w-46), yPin=p=>p?18:h-22, yH=v=>h-22-v/50*(h-40);
 cx.strokeStyle='#ffad42';cx.lineWidth=1.5;cx.beginPath();points.forEach((q,i)=>{let X=x(q.t),Y=yH(q.h);i?cx.lineTo(X,Y):cx.moveTo(X,Y)});cx.stroke();
 cx.strokeStyle='#5cff78';cx.lineWidth=2;cx.beginPath();points.forEach((q,i)=>{let X=x(q.t),Y=yPin(q.p);if(!i)cx.moveTo(X,Y);else{cx.lineTo(X,yPin(points[i-1].p));cx.lineTo(X,Y)}});cx.stroke();
}
async function poll(){
 if(paused||document.hidden){timer=setTimeout(poll,250);return} if(busy)return;busy=true;
 try{const r=await fetch('/pirdata?x='+Date.now(),{cache:'no-store'});if(!r.ok)throw 0;show(await r.json())}
 catch(e){setv('conn','помилка','bad');document.body.classList.add('offline')}finally{busy=false;timer=setTimeout(poll,200)}
}
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'Продовжити':'Пауза';if(!paused)poll()};
$('clear').onclick=()=>{points=[];lines=[];last=null;$('log').textContent='';draw()};
$('save').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/plain'}));a.download='pir-log-'+new Date().toISOString().replace(/[:.]/g,'-')+'.txt';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
$('events').onclick=async()=>{
 const codes=['ВІДКИНУТО','РУХ','НАВОДКА','ТИК','ЗАДОВГА','РУХ(вікно)'];const lampN=['OFF','ON','WAVE','ALERT'];
 try{const r=await fetch('/pirevents?x='+Date.now(),{cache:'no-store'});const j=await r.json();
  const tag=f=>[f&1?'RF':'',f&2?'BLK':'',f&4?'STK':'',f&8?'DAY':''].filter(Boolean).join(' ');
  lines.push(`--- журнал: схожих на рух ${j.a}, відкинуто ${j.r} ---`);
  (j.ev||[]).forEach(e=>{const s=Math.floor(e[0]/1000);
   lines.push(`# ${Math.floor(s/60)}:${String(s%60).padStart(2,'0')} ${e[1]}мс HIGH ${e[2]}/50 ${codes[e[3]]||e[3]} ${tag(e[4])} lvl=${e[5]*4} br=${e[6]} ${lampN[e[7]]||''}`);});
  $('log').textContent=lines.slice(-1200).join('\n');$('log').scrollTop=$('log').scrollHeight;
 }catch(e){setv('conn','журнал: помилка','bad')}
};
addEventListener('resize',draw);document.addEventListener('visibilitychange',()=>{if(!document.hidden&&!paused)poll()});poll();
</script></body></html>)HTML";
  server.send_P(200, PSTR("text/html"), PAGE);
}

void handlePirData() {
  unsigned long now = millis();
  String j;
  j.reserve(260);
  j = F("{\"ms\":"); j += String(now);
  j += F(",\"pir\":");   j += String(pirPinHigh ? 1 : 0);
  j += F(",\"hc\":");    j += String(highCount);
  j += F(",\"hn\":");    j += String(pirHighNowMs);
  j += F(",\"hi\":");    j += String(pirLastHighMs);
  j += F(",\"mot\":");   j += String(motionNow ? 1 : 0);
  j += F(",\"gate\":");  j += String(pirGated(now) ? 1 : 0);
  j += F(",\"blk\":");   j += String(pirRunBlocked ? 1 : 0);
  j += F(",\"rf\":");    j += String(pirRunFalse ? 1 : 0);
  j += F(",\"stuck\":"); j += String(pirStuck ? 1 : 0);
  j += F(",\"ev\":");    j += String(pirEvents);
  j += F(",\"rn\":");    j += String(radioNoiseRuns);
  j += F(",\"lamp\":");  j += String((int)lampState);
  j += F(",\"warm\":");  j += String((now - bootAt) < warmupMs ? 1 : 0);
  j += F(",\"hl\":");    j += String(cfg.holdSec);
  j += F(",\"why\":\"");  j += lampWhy; j += '"';
  j += F(",\"bc\":");    j += String(bootCount);
  j += F(",\"rr\":\"");  j += jsonEscape(resetReason); j += '"';
  j += F(",\"up\":");    j += String((now - bootAt) / 1000UL);
  j += F(",\"blank\":"); j += String((long)(now - pirBlankUntil) < 0 ? 1 : 0);
  j += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// Журнал останніх серій PIR. Викликається вручну з /pir (або /pirevents),
// щоб не плодити Wi-Fi-трафік, який сам же створює наводки.
void handlePirEvents() {
  String j;
  j.reserve(90 + pirLogCount * 64);
  j = F("{\"a\":"); j += String(pirAccepted);
  j += F(",\"r\":"); j += String(pirRejected);
  j += F(",\"ev\":[");
  uint8_t idx = (uint8_t)((pirLogHead + PIR_LOG_N - 1) % PIR_LOG_N);
  for (uint8_t i = 0; i < pirLogCount; i++) {
    PirLogEntry &e = pirLog[idx];
    if (i) j += ',';
    j += '['; j += String((unsigned long)e.startMs);
    j += ','; j += String((unsigned)e.durMs);
    j += ','; j += String((unsigned)e.hc);
    j += ','; j += String((unsigned)e.code);
    j += ','; j += String((unsigned)e.flags);
    j += ','; j += String((unsigned)e.lvl);
    j += ','; j += String((unsigned)e.br);
    j += ','; j += String((unsigned)e.lamp);
    j += ']';
    idx = (uint8_t)((idx + PIR_LOG_N - 1) % PIR_LOG_N);
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void handleForget() {
  clearCreds();
  WiFi.setAutoReconnect(false);   // інакше SDK може одразу перепідключитись
  WiFi.disconnect(false);
  connectPending = false;
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleNotFound() {
  if (requestFromAP()) { redirectToPortal(); return; }
  // captive portal: будь-який запит -> на головну (відносний шлях працює і через AP, і через LAN)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ---------- AP вмикається / вимикається ----------
void apStart() {
  if (apRunning) return;

  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  connectPending = false;
  apJoinPending = false;
  apJoinApplyAt = 0;
  WiFi.setOutputPower(17.0f);  // setup needs reliable phone coverage; restored when AP stops
  // STA ще не підключено: шукаємо канал нашого роутера скануванням.
  // ESP8266 STA+AP працюють лише на одному каналі, ch=1 блокиє підключення.
  if (!netsValid || (haveCreds && (staChannel < 1 || staChannel > 14))) {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      bool m = (WiFi.SSID(i) == String(creds.ssid));
      Serial.print("AP scan[");
      Serial.print(i);
      Serial.print("] ch=");
      Serial.print(WiFi.channel(i));
      Serial.print(" rssi=");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" match=");
      Serial.println(m ? 1 : 0);
      if (m) { staChannel = WiFi.channel(i); }
    }
    collectNets(n);  // cache the list BEFORE the phone joins the AP
    Serial.print("AP scan: ");
    Serial.print(n);
    Serial.print(" мереж, канал ");
    Serial.println(staChannel);
  }
  uint8_t ch = (staChannel >= 1 && staChannel <= 14) ? staChannel : 1;
  const IPAddress portalIP(192, 168, 4, 1);
  if (!WiFi.softAPConfig(portalIP, portalIP, IPAddress(255, 255, 255, 0)) ||
      !WiFi.softAP(apSsidStr.c_str(), AP_PASSWORD, ch)) {
    Serial.println("AP: start failed; will retry");
    return;
  }
  if (!dnsServer.start(53, "*", WiFi.softAPIP()))
    Serial.println("AP: DNS start failed; direct http://192.168.4.1/ remains available");
  WiFi.mode(WIFI_AP);  // stable setup radio; STA joins only after pressing Connect

  apRunning = true;
  apStopAt = 0;

  Serial.println();
  Serial.print("AP ON: ");
  Serial.print(apSsidStr);
  Serial.print("  pass: ");
  Serial.print(AP_PASSWORD);
  Serial.print("  ch=");
  Serial.print(ch);
  Serial.print("  http://");
  Serial.println(WiFi.softAPIP());
}

void apStop() {
  if (!apRunning) return;

  dnsServer.stop();
  WiFi.softAPdisconnect(true);   // точка доступу більше не роздається
  WiFi.mode(WIFI_STA);           // назад у чистий STA
  WiFi.setAutoReconnect(true);
  WiFi.setOutputPower(WIFI_TX_DBM);
  apRunning = false;
  apStopAt = 0;

  Serial.print("AP OFF (підключено до ");
  Serial.print(WiFi.SSID());
  Serial.print(", IP ");
  Serial.print(WiFi.localIP());
  Serial.println(") — портал доступний за цим IP");
}

// ---------- старт мережі ----------
void wifiBegin() {
  loadCreds();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);   // без modem-sleep: на слабкому сигналі DTIM дає багатосекундні залипання TCP
  WiFi.setOutputPower(WIFI_TX_DBM);     // менше RF-наводки на PIR (D5 поруч з антеною)

  apSsidStr = String(AP_PREFIX) + String(ESP.getChipId(), HEX);

  // Ім'я в мережі унікальне для кожного пристрою: два нічники в одній квартирі
  // не перебивають один одного, а ім'я можна назвати покупцю для підтримки.
  char idbuf[8];
  snprintf(idbuf, sizeof(idbuf), "%04x", (unsigned)(ESP.getChipId() & 0xFFFF));
  mdnsHost = String(MDNS_PREFIX) + "-" + idbuf;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/connect", HTTP_POST, handleConnect);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/wifi-networks", HTTP_GET, handleAPNetworks);
  lanRoute("/live", HTTP_GET, handleLive);
  lanRoute("/pir", HTTP_GET, handlePirPage);
  lanRoute("/pirdata", HTTP_GET, handlePirData);
  lanRoute("/pirevents", HTTP_GET, handlePirEvents);
  lanRoute("/forget", HTTP_GET, handleForget);
  lanRoute("/save", HTTP_POST, handleSave);
  lanRoute("/reset", HTTP_POST, handleReset);
  lanRoute("/schedsync", HTTP_POST, handleSchedSync);
  lanRoute("/alertsync", HTTP_POST, handleAlertSync);
  lanRoute("/alerttest", HTTP_GET, handleAlertTest);
  lanRoute("/beztest", HTTP_GET, handleBezTest);
  lanRoute("/reboot", HTTP_POST, handleReboot);
  lanRoute("/wifitest", HTTP_GET, handleWifiTest);
  // OS connectivity checks must redirect to a numeric, HTTP-only portal URL.
  const char* probes[] = {"/generate_204", "/gen_204",
    "/connecttest.txt", "/ncsi.txt", "/redirect", "/canonical.html", "/success.txt"};
  for (const char* path : probes) server.on(path, HTTP_GET, []() {
    if (requestFromAP()) redirectToPortal();
    else { server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
  });
  // iOS CNA is less fragile when its initial HTTP probe gets the setup page
  // directly, with no extra redirect round-trip. Never return Apple's Success page.
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    if (requestFromAP()) pageAPWiFi();
    else { server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
  });
  server.on("/library/test/success.html", HTTP_GET, []() {
    if (requestFromAP()) pageAPWiFi();
    else { server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
  });
  server.onNotFound(handleNotFound);
  server.begin();

  // Existing credentials get a STA-only attempt first. Without credentials,
  // start the setup portal immediately; failed saved connections get a fallback AP.
  staDownTracked = true;
  staDownSince = millis();

  if (haveCreds) {
    WiFi.begin(creds.ssid, creds.pass);
    connectPending = true;
    connectStartedAt = millis();
    staRetryAt = connectStartedAt + WIFI_TIMEOUT;
    Serial.print("STA -> ");
    Serial.println(creds.ssid);
  } else {
    Serial.println("STA: мережа ще не збережена");
    apStart();  // first-time setup must be available immediately, not after 15 s
  }
}

// ---------- контроль AP/STA ----------
void mdnsStart() {
  if (mdnsRunning) return;
  if (MDNS.begin(mdnsHost.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.print("mDNS: http://");
    Serial.print(mdnsHost);
    Serial.println(".local/");
  } else {
    Serial.println("mDNS: не вдалося запустити");
  }
}

void mdnsStop() {
  if (!mdnsRunning) return;
  MDNS.end();
  mdnsRunning = false;
}

void wifiStateTick() {
  unsigned long now = millis();
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected) {
    connectPending = false;
    apJoinPending = false;
    staDownTracked = false;
    staChannel = WiFi.channel();   // канал роутера (для AP: той самий канал)
    mdnsStart();

    // Give the phone time to receive the home IP, then close the setup AP
    // regardless of whether the phone is still associated.
    if (apRunning) {
      if (apStopAt == 0) apStopAt = now + AP_LINGER_MS;
      else if (timeReached(now, apStopAt)) apStop();
    }
  } else {
    mdnsStop();

    // STA повторюємо раз на STA_RETRY_MS: після збою роутера драйвер сам не відновлюється
    if (!apRunning && haveCreds && timeReached(now, staRetryAt)) {
      staRetryAt = now + STA_RETRY_MS;
      WiFi.begin(creds.ssid, creds.pass);
      Serial.print("STA: retry (");
      Serial.print(WiFi.status());
      Serial.println(")");
    }

    // ДАГОСЛИТ: один раз після довгого провалу (скан рве поточне підключення,
    // тому робимо його лише коли STA немає > 60 с і не частіше ніж раз на 60 с)
    if (!apRunning && haveCreds && (now - staDownSince > 60000UL) && timeReached(now, staScanAt)) {
      staScanAt = now + 60000UL;
      int n = WiFi.scanNetworks();
      Serial.print("SCAN: n=");
      Serial.print(n);
      Serial.print(" want=");
      Serial.print(creds.ssid);
      Serial.print(" passlen=");
      Serial.println(strlen(creds.pass));
      for (int i = 0; i < n && i < 6; i++) {
        Serial.print("  [");
        Serial.print(i);
        Serial.print("] ch=");
        Serial.print(WiFi.channel(i));
        Serial.print(" rssi=");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" enc=");
        Serial.print((int)WiFi.encryptionType(i));
        Serial.print(" ");
        Serial.print(WiFi.SSID(i));
        if (WiFi.SSID(i) == creds.ssid) Serial.print(" <<WANT>>");
        Serial.println();
      }
    }

    // A bad password must not leave the SDK roaming/retrying under the portal.
    // Retain connectPending so /status can report the failed attempt to the UI.
    if (apRunning && apJoinPending && now - connectStartedAt >= WIFI_TIMEOUT) {
      WiFi.disconnect(false);
      WiFi.mode(WIFI_AP);
      apJoinPending = false;
      Serial.println("AP: join timed out; stable setup mode restored");
    }
    if (apRunning) apStopAt = 0;

    if (!staDownTracked) {
      staDownTracked = true;
      staDownSince = now;
    }
    // Recovery must remain accessible even after a router/password change.
    const unsigned long portalDelay = haveCreds ? 30000UL : 0UL;
    if (!apRunning && apStopAt == 0 && (now - staDownSince >= portalDelay)) {
      apStart();
    }
  }
}

void wifiLoop() {
  unsigned long now = millis();

  // ДАГОСЛИТ «радіо вимкнено»: застосувати/відновити
  if (wifiOffApplyAt && timeReached(now, wifiOffApplyAt)) {
    wifiOffApplyAt = 0;
    WiFi.mode(WIFI_OFF);
    Serial.print("WIFITEST: RADIO OFF @");
    Serial.print(now);
    Serial.print(" подій PIR=");
    Serial.println(pirEvents);
  }
  if (wifiOffUntil && timeReached(now, wifiOffUntil)) {
    wifiOffUntil = 0;
    WiFi.mode(WIFI_AP_STA);
    if (haveCreds) WiFi.begin(creds.ssid, creds.pass);
    Serial.print("WIFITEST: RADIO ON @");
    Serial.print(now);
    Serial.print(" подій PIR=");
    Serial.println(pirEvents);
  }

  if (apRunning) dnsServer.processNextRequest();
  server.handleClient();
  if (apJoinApplyAt && timeReached(now, apJoinApplyAt)) {
    apJoinApplyAt = 0;
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect(false);
    WiFi.begin(creds.ssid, creds.pass);
    connectStartedAt = millis();
    staRetryAt = connectStartedAt + WIFI_TIMEOUT;
  }
  if (wifiOffApplyAt || wifiOffUntil) return;   // під час тесту STA/AP логіку не чіпаємо
  wifiStateTick();
  if (mdnsRunning) MDNS.update();
}

// ================= SETUP =================
void setup() {
  bootAt = millis();

  settingsLoad();   // налаштування з EEPROM (або значення за замовчуванням)

  // Діагностика перезавантажень: раптове згасання лампи часто = brownout/reboot,
  // а не «рух зник». Лічильник живе в RTC-пам'яті (переживає soft-reset).
  resetReason = ESP.getResetReason();
  uint32_t bc = 0;
  ESP.rtcUserMemoryRead(0, &bc, sizeof(bc));
  if (bc == 0xFFFFFFFFUL) bc = 0;
  bc++;
  ESP.rtcUserMemoryWrite(0, &bc, sizeof(bc));
  bootCount = bc;
  // HC-SR501 потребує прогріву лише після справжнього знеструмлення.
  warmupMs = (unsigned long)cfg.warmupSec * 1000UL;
  if (resetReason != "Power on") warmupMs = 3000UL;

  Serial.begin(115200);

  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TZ_INFO, 1);
  tzset();
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_VIB, OUTPUT);
  digitalWrite(PIN_VIB, LOW);

  resetWindow();

  ring.begin();
  ring.clear();
  ring.show();

  // стартове значення яскравості одразу з датчика
  ldrRaw = readLdr();
  ldrSmooth = ldrRaw;
  currentBrightness = targetBrightness = brightnessForLdr(ldrSmooth);

  // HC-SR501 після прогріву має показати стабільний LOW перед першим рухом.

  Serial.println("SMART NIGHT LIGHT START");

  wifiBegin();   // AP + веб-портал для налаштування Wi-Fi
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  updateBrightness();
  updateMotion();
  updateLeds();
  updateVibe();
  wifiLoop();
  schedTick();
  alertTick();

  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("LDR: ");
    Serial.print(ldrRaw);
    Serial.print(" | BR: ");
    Serial.print(currentBrightness);
    Serial.print(" | PIR: ");
    Serial.print(digitalRead(PIN_PIR));
    Serial.print(" | HIGH: ");
    Serial.print(highCount);
    Serial.print("/");
    Serial.print(WINDOW_SAMPLES);
    Serial.print(" | MOT: ");
    Serial.print(motionNow ? 1 : 0);
    Serial.print(" | STUCK: ");
    Serial.print(pirStuck ? 1 : 0);
    Serial.print(" | GATE: ");
    Serial.print(pirGated(now) ? 1 : 0);
    Serial.print(" | RF: ");
    Serial.print(pirRunFalse ? 1 : 0);
    Serial.print(" | RN: ");
    Serial.print(radioNoiseRuns);
    Serial.print(" | LVL: ");
    Serial.print(lightLevelFor(ldrSmooth));
    Serial.print(" | DAY: ");
    Serial.print(daylight ? 1 : 0);
    Serial.print(" | WARMUP: ");
    Serial.print(((now - bootAt) < warmupMs) ? 1 : 0);
    Serial.print(" | ARM: ");
    Serial.print(isReallyDark() ? 1 : 0);
    Serial.print(" | STA: ");
    Serial.print((WiFi.status() == WL_CONNECTED) ? 1 : 0);
    Serial.print(" RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.print(" | STATE: ");
    Serial.println(stateName(lampState));
  }

  // раз на 10 с нагадуємо стан мережі
  if (now - lastApPrint >= 10000) {
    lastApPrint = now;
    if (apRunning) {
      Serial.print("AP: ");
      Serial.print(apSsidStr);
      Serial.print(" | pass: ");
      Serial.print(AP_PASSWORD);
      Serial.print(" | http://");
      Serial.print(WiFi.softAPIP());
      Serial.print(" | clients: ");
      Serial.println(WiFi.softAPgetStationNum());
    } else {
      Serial.print("AP off | STA IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(" | http://");
      Serial.print(mdnsHost);
      Serial.print(".local/ | portal: http://");
      Serial.print(WiFi.localIP());
      Serial.println("/");
    }

    Serial.print("SCHED: ");
    if (!cfg.schedEnabled) {
      Serial.println("off");
    } else {
      Serial.print("grp=");   Serial.print(cfg.schedGroup);
      Serial.print(" valid="); Serial.print(schedValid ? 1 : 0);
      Serial.print(" date=");  Serial.print(schedDay);
      Serial.print(".");       Serial.print(schedMon);
      Serial.print(".");       Serial.print(schedYear);
      Serial.print(" slot=");  Serial.print(schedSlotNow);
      Serial.print(" out=");   Serial.print(schedSlotOut ? 1 : 0);
      Serial.print(" allow="); Serial.print(schedAllowedNow ? 1 : 0);
      Serial.print(" err=");   Serial.print(schedErr);
      Serial.print(" heap=");  Serial.println(ESP.getFreeHeap());
    }

    Serial.print("ALERT: ");
    if (!cfg.alertEnabled) {
      Serial.println("off");
    } else {
      Serial.print("r=");      Serial.print(cfg.alertRaion);
      Serial.print(" lvl=");   Serial.print(cfg.alertLastLevel);
      Serial.print(" notify=");Serial.print(alertNotify);
      Serial.print(" err=");   Serial.print(alertErr);
      Serial.print(" poll=");  Serial.print(cfg.alertPollSec);
      Serial.print("s since=");Serial.print(alertSince);
      Serial.print(" heap=");  Serial.println(ESP.getFreeHeap());
    }
  }

  delay(2);
}

