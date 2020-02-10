 # Robot dance

Tento program vznikl jako úkol do [Úvodu do robotiky na MFF UK](http://ktiml.mff.cuni.cz/~obdrzalek/vyuka/NAIL028/). Cílem úkolu bylo vytvořit robota, který zvládne vykonávat pohyby na čtvercové síti podle daných instrukcí (choreografie).

Robot obdrží na začátku startovní pozici a popis toho, jak se má následně přemísťovat. U každého přemístění je omezen minimální čas od spuštění choreografie, kdy lze pokračovat s další instrukcí. Robot je schopný vykonávat danou choreografii a v libovolnou chvíli se začít vracet do výchozí polohy. 

Robot umí uložit až 5 choreografií do EEPROM paměti a označit tu, která má být výchozí při zapnutí robota. Po zapnutí robota je načten plán z výchozího EEPROM slotu. Pokud je z nějakého důvodu prázdný, tak je použit výchozí vestavěný plán.

## Ovládání robota

Po zapnutí robot přijímá příkazy po sériové lince. Po stisknutí tlačítka robot začne vykovávat nastavenou choreografii. Po dalším stisknutí tlačítka (v libovolný čas) se robot začne vracet na výchozí pozici.

### Komunikace přes sériovou linku

Před stiskem tlačítka lze robota ovládat přes sériovou linku sadou jednoduchých příkazů, přes kterou navíc robot hlásí chyby. Každý příkaz se musí skládat přesně ze dvou bajtů, první je interpretován jako písmeno značící příkaz, druhý jako číslice `N=0-9`. Nejsou dovoleny žádné extra mezery nebo jiné znaky, s výjimkou příkazu S. Písmeno musí být velké.

**Příkazy dle písmena:**

- `C` - Vynuluje EEPROM paměť pro n-tý slot a jeho hlavičku.
- `D` - Nastaví n-tý slot jako výchozí.
- `S` - Tento příkaz musí být následován alespoň jednou mezerou a choreografií v formátu daném zadáním, pokud je plán načtený úspěšně, tak je uložen do n-tého slotu a zároveň je načten. V opačném případě pošle robot chybovou hlášku a n-tý slot ani aktivní plán se nezmění.
- `L` - Načte plán uložený v n-tém, slotu. Pokud je prázdný, tak pošle upozornění. Příkaz se nesmí volat na slot, na který nebyl dříve použit příkaz S nebo E nebo pokud není zaručeno, že je pamět vynulovaná.

Pokud je poslán příkaz S, tak další příkaz je možno poslat až když jej robot zpracuje a informuje o tom. Ostatní jdou poslat najednou.

Kvůli implementaci je doporučeno před prvním použitím nového robota zavolat sérii příkazů `C0C1C2C3C4D0` a robota pak restartovat. Případně zaručit, že je paměť vynulovaná. Robot totiž počítá s tím, že EEPROM obsahuje validní hlavičku a korektní choreografie. Pokud to tak není, tak je chování robota bez explicitního úspěšného načtení nového plánu (příkaz S) nepředvídatelné.

## Vývojová dokumentace

Pro překlad je vyžadováno [Arduino IDE](https://www.arduino.cc/en/main/software) (doporučená verze 1.8.11). Program se překládá z hlavního souboru `robot_dance.ino`. 

Program je rozdělen na _Plánovací část_ a _Exekuční část_. Plánovací část se stará o zpracování plánů, příjem plánů ze sériové linky, plánování návratu robota a rozdělení plánu na __základní příkazy__:

- Jeď o jedno políčko dopředu
- Otoč se doleva / otoč se doprava
- Čekej
- Choreografie dokončena

Exekuční část pak tyto příkazy vykonává. Exekuce také řídí životní cyklus robota a čte jeho senzory.


### Plánovací část

**Plánování zahrnuje:**
- Načítání plánů řízené uživatelem přes sériovou linku před spuštěním choreografie (stiskem tlačítka), o to se stará třída Planner. Umožňuje načíst až 5 plánů do EEPROM pamětí, takže choreografie vydrží i po vypnutí.
- Aktivaci jednoho plánu, což znamená vytvoření instance třídy Plan, se kterým pak po stisku tlačítka pracuje exekuční část. Aby byla exekuční část co nejjednodušší, tak instance třídy Plan je schopná dávat robotovi jednoduché instrukce WAIT, LEFT, RIGHT, GO nebo FINISHED na základě aktivního plánu a času od spuštění.

Obě části robota jsou tedy na sobě dost nezávislé a komunikují spolu pouze předáváním instancí třídy Plan.

Protože choreografie uložená ve stringu zabírá alespoň 5B při dvouciferném času, více přidáním mezer, tak je každý přijatý plán předzpracován do instrukcí, kde každá zabírá 3B. V této formě je uložen aktivní plán
v RAM i plány v EEPROM.


#### Třída Plan
Instance této třídy vždy reprezentuje aktivní plán. Exekuční část o tuto instanci požádá statickou třídu Planner a instance je platná do té doby než je požádáno o nový aktivní plán.  Po jejím vytvoření se předpokládá, že robot se nachází na pozici, která je určena v načtené choreografii.

Tenhle přístup byl zvolen proto, že nedává smysl mít více plánů rozeběhnutých zároveň protože by došlo k desynchronizaci mezi skutečnou a předpokládanou polohou robota. Navíc tento přistup dovoluje to, že Plan nemusí mít dynamicky alokované žádné pole, protože existuje jedno globální statické pro aktivní plán, o které se stará Planner.

**Třída poskytuje následující dvě funkce:**

- `getNext(time)` - Vrací instrukce dle aktivního plánu.
- `goHome()` - Vrací instrukce, které dovedou robota do jeho výchozí pozice.

Obě fungují tak, že když je exekuční část zavolá tak od nich dostane jednu z možných instrukcí popsaných výše, kterou vykoná a poté může požádat o další. `goHome` nepotřebuje čas a nikdy nevrací WAIT. 

Důležité je, že robot vždy musí danou instrukci vykonat, další volání s tím počítá a vrací už další instrukci. `Plan` si totiž udržuje aktuální pozici a orientaci robota, kterou mění při vracení instrukce. Když se toto dodrží tak se `goHome` může začít volat kdykoliv. Pokud už robot vykonal celý plán, případně se vrátil domů, tak obě funkce začnou vracet FINISHED.

#### Třída Planner
Třída `Planner` se stará o vytváření plánů a komunikaci s uživatelem, což znamená naparsování a uložení plánů ze sériové komunikace a práci s EEPROM, ve které mohou být uložený již předzpracované plány. Parsování je navrhnuto obecně nad rozhraním podobné streamům, takže je případně možné přidat nové zdroje plánů. Proto třída dovoluje i přímo načítat plán z řetězce v kódu, což sloužilo spíše pro testování.

Celá třída je navržena jako statická, protože by stejně nemohla rozumně existovat vícekrát současně.

##### Formát předzpracovaného plánu

`Planner` si udržuje dvě statická pole pro aktivní a rozparsovaný plán, celkem cca 400B.
To druhé pole je nutné k tomu, že pokud by parsování plánu někde uprostřed selhalo, tak aby aktivní plán zůstal nezměněn.

1. Struktura s výchozí konfigurací robota:
    - jeho poloha jako řádek a sloupec = 2x1B
    - orientace - enum(N,W,E,S) = 1B
2. počet instrukcí N<=64 = 1B
3. N 3B struktur reprezentující instrukce:
    - řádek, sloupec cíle - 2x4b = 1B
    - startovací čas, omezen na 6353,5 sekund = 15b
    - flag jestli se má nejdřív dosáhnout cílového řádku nebo sloupce = 1b

Maximální velikost celého plánu je 64x3B + 4B + 1B = 197B. Limit na 64 instrukcí na plán se dá změnit, ale limitující faktor je hlavně 1KB EEPROM, takže při zvětšení plánu je potřeba případně upravit počet slotů.

##### Obsah EEPROM
1. Číslo výchozího slotu = 1B
2. Hlavičky slotů - 5x struktura:
    - výchozí konfigurace robota = 4B
    - počet instrukcí = 1B
3. 5x pole délky \<Maximální délka plánu> s 3B stukturama reprezentujícíma instrukce.

Celkem je při 5 slotech s délkou plánů 64 zabráno 1B + 5x5B + 5x64x3B=986B EEPROM.

### Exekuční část

Životní cyklus robota se nachází v souboru `robot_dance.ino`. Životní cyklus je pak definován jako konečný automat, který přechází mezi následujícími  stavy:

- Čekání na spuštění, kdy se přijímají stavy ze sériové linky.
- Exekuce, kdy se provádí zvolený plán.
- Návrat domů, kdy se de facto provádí plán návratu.

Mezi jednotlivými stavy se přechází stisknutím tlačítka. V každém průchodu hlavní smyčkou `loop()` se přečte stav senzorů a tlačítka. Podle aktuálního stavu se pak tyto data předají do patřičné třídy k dalšímu zpracování. 

#### Knihovna `ctl_common.h`

Knihovna definuje společné objekty pro exekuční část. Jedná se hlavně o abstrakci motoru `Motor`, která na rozdíl od třídy `Servo`, umožňuje změnit směr otáčení a používat procentuální určení rychlosti.

Pak knihovna obsahuje strukturu `sensors_t`, která slouží pro předávání dat z optických senzorů.

#### Knihovna `execution_task.h`

Knihovna definuje třídu `ExecutionTask`, která slouží pro vykonávání základních příkazů. Třída se zeptá plánovače, který příkaz má v daném čase vykonávat. Pak tento příkaz vykonává dokud není dokončen. Pokud je dokončen, tak se třída dotáže plánovače na následující příkaz.

V rámci každého průchodu hlavní smyčkou se volá metoda `tick(...)`, která následně volá aktuální základní příkaz. Pokud je příkaz dokončen, tak se už v rámci metody `tick(...)` dotáže třída metodou `featchNextStep(...)` plánovače na další základní akci. Při tomto dotazu se případně respektuje požadavek hlavního cyklu na návrat na výchozí pozici (návrat na výchozí pozici proto začíná vždy až po dokončení posledního základního příkazu).

Základní příkazy jsou pak definovány jako potomci třídy `PlanStep`. Každý potomek definuje svoji vlastní metodu `tick(...)` a metodu `isDone()` pro zjištění zda už daná úloha byla dokončena. 

- Posun o jedno políčko je definován jako sledování čáry, které čeká na detekování konce políčka (= černé barvy na libovolných dvou krajních senzorech najednou). Po detekci čáry robot ještě pokračuje po konstantní časový úsek, aby skončil s koly na čáře.

- Rotace přijímá v konstruktoru parametr, který určuje směr rotace. Pak se jedná o konečný automat, který přechází pomocí senzorů mezi stavy _start (střed na čáře)_ -> _čára mimo střed_ -> _čára mimo senzory_ -> _znovu čára na středu_.

- Čekací příkaz pak nic neprovádí a je okamžitě dokončen. Slouží pro vykrytí doby, kdy plánovač nemůže poskytnou další příkaz než uplyne požadovaný čas, kdy je možné pokračovat.