# Power-outage schedule sources

This is a list of firmware integrations, **not a guarantee of data availability or freshness**. Websites, APIs, and payload formats can change independently of this project.

## Primary sources

| Region | Source |
|---|---|
| Chernivtsi | `oblenergo.cv.ua/shutdowns/` |
| Kyiv city, Kyiv region, Dnipropetrovsk, Odesa | [Baskerville42/outage-data-ua](https://github.com/Baskerville42/outage-data-ua) |
| Vinnytsia | [vn-progr/gpv-voe-vinnytsia](https://github.com/vn-progr/gpv-voe-vinnytsia) |
| Cherkasy, Chernihiv, Kharkiv, Khmelnytskyi, Lviv, Poltava, Ivano-Frankivsk, Rivne, Ternopil, Zakarpattia, Zaporizhzhia, Zhytomyr | [yaroslav2901/OE_OUTAGE_DATA](https://github.com/yaroslav2901/OE_OUTAGE_DATA) |
| Mykolaiv | `off.energy.mk.ua/api/v2/schedule/active` |
| Kirovohrad | Public Telegram posts from `@SvitloKropyvnytskyiMisto`, without a personal Telegram API key |
| Sumy | `soe.com.ua/next-api/shutdowns/schedules` |

The code lists 21 sources, counting Kyiv city separately from Kyiv region. Not every source necessarily publishes current data. Volyn, Donetsk, Luhansk, and Kherson are not currently included.

## Setup

1. Enable the option to operate only during scheduled power outages.
2. Select the region and queue/group. Choices depend on the source: Chernivtsi uses groups `1..12`, Kyiv city uses `1.1..60.1`, and most others use `1.1..6.2`.
3. Save and request a schedule update.
4. Check the data date and status in the web interface. Adjust the unavailable-schedule policy if needed.

Fail-open is enabled by default: if the schedule is unavailable, the night light may operate normally. This is not confirmation of an actual mains outage.

## Data handling

- A day is stored as 48 half-hour slots in a six-byte bitmask.
- Mirrors with 24 hourly values are streamed and converted to half-hour slots without retaining the entire JSON response in RAM.
- `maybe`, `mfirst`, and `msecond` states depend on the option to count possible outages (the UI calls this «МЗ»).
- Mykolaiv: `OFF`, `SURE_OFF`, and `PROBABLY_OFF`; the last requires the possible-outage option.
- Kirovohrad: text intervals such as `Черга 1.1: 08-10, 16-18`, including `HH:MM-HH:MM` formats.
- Sumy: a base64 payload with encrypted data from the public website client, streamed through decoding and parsing. The current implementation does not verify the GCM tag; this is a known integrity limitation, not a complete authenticated AES-GCM implementation.

## Fallback source

When the primary source does not provide usable fresh data, the firmware tries `bezsvitla.com.ua/<region>/cherha-<queue>`. It streams the HTML attributes `data-schedule-date` and `data-schedule-intervals`, converting `OFF` intervals into slots.

The diagnostic endpoint `http://<ip>/beztest?src=7&q=1.1` writes its result to Serial. The `src` index refers to the source list in the sketch; verify the index before use.

Successful refreshes are scheduled every 15 minutes, with retries usually two minutes after failure. Requests can block the main loop. Schedules are forecasts, not measurements of mains voltage.
