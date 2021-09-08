# Autoelektronika Projekat - Simulacija LIN Komunikacije

## Uvod

Ovaj projekat ima za zadatak da simulira LIN komunikaciju. Kao okruženje koristi se VisualStudio2019. Zadatak ovog projekta osim same funkcionalnosti je bila i implementacija Misra standarda prilikom pisanja koda.

## Ideja i zadaci: 
  1. Pritiskom na jedan od tri tastera LED bara, salje se adresa tog senzora preko LIN komunikacije i senzor vraca vrijednost. Senzor ce slati vrijednost svakih 100ms sve dok je taster pritisnut.
  2. Komunikacija sa PC-jem, izvršena je simuliranjem LIN komunikacije preko serijske komunikacije kanala 1. Sa PC-a se šalje naredba kojom se traži podatak od odredjenog senzora. Zatim se PC-u šalje odgovor koji predstavlja vrijednost tog senzora.
  3. Na LCD displeju (koji se osvježava svakih 100ms), prikazuje se adresa senzora od kojeg su zatraženi podaci, kao i vrijednost tog senzora.

## Periferije

Periferije koje je potrebno koristiti su LED_bar, 7seg displej i AdvUniCom softver za simulaciju serijske komunikacije.
Prilikom pokretanja LED_bars_plus.exe navesti g kao argument da bi se dobio led bar sa 1 ulaznim stupcem zelene boje.
Prilikom pokretanja Seg7_Mux.exe navesti kao argument broj 10, kako bi se dobio 7-seg displej sa 10 cifara (potrebno je 18 cifara ali smo ograničeni jer je moguće pokrenuti displej sa maksimalno 10 cifara).
Što se tiče serijske komunikacije, potrebno je otvoriti kanale 1, 2, 3 i 4. Kanali se otvaraju pisanjem sledećeg teksta u cmd, redom: AdvUniCom 1, AdvUniCom 2, AdvUniCom 3 i AdvUniCom 4.

## Kratak pregled taskova

Glavni .c fajl ovog projekta je main_application.c

### static void led_bar_tsk(void* pvParameters)

Task koji na osnovu pritisnutog tastera na led baru salje adresu odredjenom senzoru. Skroz donji taster - adresa senzora 0x07, srednji taster - adresa senzora 0x08 i treci taster - adresa senzora 0x09. Poruke koje se salju su u obliku LIN poruka odnosno frame - ova, tako da se npr. za adresu 0x07 salje poruka oblika 005507. Poruka se salje u dva reda, jedan se koristi za ispis adrese na 7seg displej a drugi za slanje u taskove koji simuliraju vrijednosti senzora.

### static void parsiranje_tsk(void* pvParameters)

Task za parsiranje. Ovaj task se koristi za prijem adrese senzora (sa PC-a ili tastera LED bara), koji na osnovu primljene adrese dodjeljuje odredjen binarni semafor.

### static void ispisAdreseNa7seg_tsk(void* pvParameters)

Task koji ispisuje poslednju zatrazenu adresu senzora na 7seg displej.

### static void ispisVrijednostiSenzoraNa7seg_tsk(void* pvParameters)

Task koji ispisuje vrijednost senzora sa zatrazene adrese na 7seg displej.

### static void slanjeNaPC_tsk(void* pvParameters)

Task za slanje vrijednosti senzora na PC. Kad primi red, smjesta ga u globalnu promjenljivu koja se onda preko taska SerialSent_Task1 salje na PC. On takodje ceka da dobije binarni semafor od taska za prijem poruka od PC-a. Ovo je potrebno da ne bi doslo do zabune odnosno da se u globalnu promjenljivu ne smjesti podatak dobijen kada je vrijednost zatrazena od senzora sa tastera LED bara

### static void SerialReceive_Task1(void* pvParameters)

Task koji se koristi za prijem adrese senzora sa PC-a. Ovaj task ujedno ima i provjeru za LIN poruku tako da automatski provjerava da li pristigla poruka ima oblik LIN poruke (npr. 005507)

### static void SerialSend_Task1(void* pvParameters)

Task koji se koristi za slanje vrijednosti senzora na PC. Kad se sa PC-a posalje adresa senzora, odnosno zatrazi vrijednost odredjenog senzora, ovim taskom se ta vrijednost salje na PC.

### static void SerialReceive_Task2(void* pvParameters)

Task koji se koristi za prijem vrijednosti senzora.  Ovaj senzor ima adresu 0x07 i simuliran je preko serijske komunikacije odnosno AdvUniCom kanala 2.

### static void SerialSend_Task2(void* pvParameters)

Task koji se koristi za simulaciju slanja senzora. Kada se salje adresa 0x07, ovaj task salje broj 7 na serijsku komunikaciju kanal 2, tako da se moze simulirati automatski odgovor koji predstavlja vrijednost senzora.

### preostali taskovi za serijsku komunikaciju (kanal 3 i 4)

Taskovi i za slanje i prijem poruka rade potpuno isto kao i taskovi za kanal 2, s tim da se salju brojevi 8 i 9, na kanale 3 i 4.

## Predlog simulacije cijelog sistema

Prvo otvoriti sve periferije kao što je opisano u tekstu iznad. Nakon toga pokrenuti simulaciju programa. 
U prozoru AdvUniCom kanala 2, upisati da kada stigne karakter '7', kanal šalje naredbu npr. S77117711. (S označava početak poruke, dok . označava kraj poruke. Čekirati Auto i ok1. Na taj način, šalje se LIN poruka odnosno vrijednost senzora 77117711. Ovo predstavlja poruku od 4 bajta (poželjno je koristiti 4, 2 ili 1 bajt za simulaciju jer 8 bajtova nije moguće ispisati na 7seg displej.
Na sličan način se simuliraju i preostala dva senzora, preko kanala 3 i 4 simulatora AdvUniCom. Na kanalu 3 upisati da kada stigne karakter '8', kanal automatski šalje poruku npr. S88444488. Na kanalu 4 upisati da kada stigne karakter '9', kanal automatski šalje poruku npr. 99990009.
Na LED baru koristiti donja 3 tastera stupca. Nakon pritiska jednog od 3 tastera, na 7seg displeju, ispisaće se adresa senzora na dvije krajnje lijeve cifre, dok će vrijednost senzora biti ispisana na preostalih 8 cifara 7seg displeja.
Za simulaciju komunikacije sa PC-jem, prvo isključiti sve tastere na LED baru. Sa PC-a, preko simulatora AdvUniCom kanala 1, slati poruke koje imaju oblik LIN poruka. 
Oblik LIN poruka je sledeći: prvo se šalje bajt 00, nakon toga se šalje 55, onda adresa senzora i na kraju 13 (inače se šalje checksum, ali u našem slučaju je to konstantna vrijednost 13). Za potrebe simulacije slati poruke oblika npr. 00550713 (ispravna adresa) ili 00550313 (pogresna adresa). Ukoliko se pošalje jedna od tri ispravne adrese (07, 08 ili 09), FreeRTOS će vratiti vrijednost željenog senzora na PC, dok će adresa i vrijednost istog biti ispisani na 7seg displej.
