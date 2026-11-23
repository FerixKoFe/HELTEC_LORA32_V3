# Projektutasítások

- Rádiós vagy soros kommunikáció módosítása előtt olvasd el a [kommunikációs specifikációt](docs/kommunikacio.md).
- Ha a felhasználó a specifikációban módosított értékek alkalmazását kéri, kezeld ezeket célértékként. Vezesd át a kódba, a használt buildkonfigurációba és az érintett alapértékekbe. Ellenőrizd a származtatott időzítéseket, valamint a MASTER és a SLAVE kompatibilitását.
- Ha a kommunikáció szoftverből változik, frissítsd a specifikációt is. Firmware-változás után futtasd az `idf.py build` parancsot az ESP-IDF környezetben. Jelezd, ha a fordítás vagy a paneles ellenőrzés nem volt elvégezhető.
