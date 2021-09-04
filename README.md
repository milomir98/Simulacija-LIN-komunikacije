# Autoelektronika Projekat - Simulacija LIN Komunikacije

## Uvod

Ovaj projekat ima za zadatak da simulira LIN komunikaciju. Kao okruženje koristi se VisualStudio2019. Zadatak ovog projekta osim same funkcionalnosti je bila i implementacija Misra standarda prilikom pisanja koda.

## Ideja i zadaci: 
  1. Podaci o trenutnom stanju senzora u automobilu se dobijaju svakih 100ms sa kanala 0 serijske komunikacije (iz akumulatora se dobijaju vrijednosti senzora).
  2. Simulacija LIN komunikacije realizovana je preko kanala 1 serijske komunikacije. Sa PC-a se šalje naredba kojom se traži podatak od odredjenog senzora. Zatim se PC-u šalje odgovor koji predstavlja vrijednost tog senzora.
  3. Vrijednosti senzora se takođe mogu dobiti i preko LED bara. Pritiskom na određeni taster, traže se podaci sa određenog senzora.
  4. Na LCD displeju (koji se osvježava svakih 100ms), prikazuje se adresa senzora od kojeg su zatraženi podaci, kao i njegova vrijednost.

## Periferije

Periferije koje je potrebno koristiti su LED_bar, 7seg displej i AdvUniCom softver za simulaciju serijske komunikacije.
Prilikom pokretanja LED_bars_plus.exe navesti RRrr kao argument da bi se dobio led bar sa 2 izlazna i 2 ulazna stupca crvene boje.
Prilikom pokretanja Seg7_Mux.exe navesti kao argument broj 9, kako bi se dobio 7-seg displej sa 9 cifara.
Što se tiče serijske komunikacije, potrebno je otvoriti i kanal 0 i kanal 1. Kanal 0 se automatski otvara pokretanjem AdvUniCom.exe, a kanal 1 otvoriti dodavanjem broja jedan kao argument: AdvUniCom.exe 1

## Kratak pregled taskova

Glavni .c fajl ovog projekta je main_application.c


## Predlog simulacije celog sistema
