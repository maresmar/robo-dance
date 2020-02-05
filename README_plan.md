Navržený robot je schopný vykonávat danou choreografii a vrátit se do výchozí polohy.
Navíc umí uložit až 5 choreografií do EEPROM paměti a označit tu, která má být výchozí
při zapnutí robota.

Po zapnutí robota je načten plán z výchozího EEPROM slotu. Pokud je z nějakého důvodu
prázdný, tak je použit záložní jednoduchý plán ve stringu.

# Komunikace s robotem

Před stiskem tlačítka lze robota ovládat přes sériovou linku sadou jednoduchých
příkazů, přes kterou navíc robot hlásí chyby.

## Příkazy
Každý příkaz se musí skládat přesně ze dvou bajtů, první je interpretován jako písmeno značící příkaz, druhý jako číslice `N=0-9`. Nejsou dovoleny žádné leading nebo trailing mezery nebo jiné znaky, s výjimkou příkazu S. Písmeno je case-sensitive.

**Příkazy dle písmena:**

- `C` - Vynuluje EEPROM pamět pro n-tý slot a jeho hlavičku.
- `D` - Nastaví n-tý slot jako výchozí.
- `S` - Tento příkaz musí být následován alespoň jednou mezerou a choreografií v formátu daném zadáním, pokud je plán načtený úspěšně, tak je uložen do n-tého slotu a zároveň je načten. V opačném případě pošle robot chybovou hlášku a n-tý slot ani aktivní plán se nezmění.
- `L` - Načte plán uložený v n-tém, slotu. Pokud je prázdný, tak pošle upozornění. Příkaz se nesmí volat na slot, na který nebyl dříve použit příkaz S nebo E nebo pokud není zaručeno, že je pamět vynulovaná.

Pokud je poslán příkaz S, tak další příkaz je možno poslat až když jej robot zpracuje a informuje o tom. Ostatní jdou poslat najednou.

Kvůli implementaci je doporučeno před prvním použitím nového robota zavolat sérii příkazů `C0C1C2C3D0` a robota pak restartovat. Případně zaručit, že je pamět vynulovaná. Robot totiž počítá s tím, že EEPROM obsahuje validní hlavičku a korektní plány. Pokud to tak není, tak je chování robota bez explicitního úspěšného načtení nového plánu (příkaz S) nepředvidatelné.


# Plánovací část

**Plánování zahrnuje:**
- Načítání plánů řízené uživatelem přes sériovou linku před spuštěním choreografie (stiskem tlačítka), o to se stárá třída Planner. Umožňuje načíst až 5 plánů do EEPROM pamětí, takže choreografie vydrží i po vypnutí.
- Aktivaci jednoho plánu, což znamená vytvoření instance třídy Plan, se kterým pak po stisku tlačítka pracuje execution část. Aby byla exekuční část co nejjednoduší, tak instance třídy Plan je schopná dávat robotovi jednoduché instrukce WAIT, LEFT, RIGHT, GO nebo FINISHED na základě aktivního plánu a času od spuštění.

Obě části robota jsou tedy na sobě dost nezávislé a komunikují spolu pouze předáváním instancí třídy Plan.

Protože choreografie uložená ve stringu zabírá alespoň 5B při dvouciferném času, více přidáním mezer, tak je každý přijatý plán předzpracován do instrukcí, kde každá zabírá 3B. V této formě je uložen aktivní plán
v RAM i plány v EEPROM.


## Třída Plan
Instance této třídy vždy reprezentuje aktivní plán. Exekuční část o tuto instanci požádá statickou třídu Planner (např po stisknutí tlačítka) a instance je platná do té doby než je požádáno o nový aktivní plán.  Po jejím vytvoření se předpokládá, že robot se nachází na pozici, která je určena v plánu.

Tenhle přístup byl zvolen proto, že nedává smysl mít více plánů rozběhnutých zároveň, protože by došlo k desynchronizaci mezi skutečnou a předpokládanou polohou robota. Navíc tento přistup dovoluje to, že Plan nemusí mít dynamicky alokované žádné pole, protože existuje jedno globální statické pole pro aktivní plán o které se stará Planner.

**Třída poskytuje následující dvě funkce:**

- `getNext(time)` - Vrací instrukce dle aktivního plánu.
- `goHome()` - Vrací instrukce, které dovedou robota do jeho výchozí pozice.

Obě fungují tak, že když je exekuční část zavolá tak od nich dostane jednu z možných instrukcí popsaných výše, kterou vykoná a poté může požádat o další. `goHome` nepotřebuje čas a nikdy nevrací WAIT. 

Důležité je, že robot vždy musí danou instrukci vykonat, další volání s tím počítá a vrácí už další instrukci. `Plan` si totiž udržuje aktuální pozici a orientaci robota, kterou mění při vracení instrukce. Když se toto dodrží tak se můžou obě funkce kombinovat. Pokud už robot vykonal celý plán, případně se vrátil domů, tak obě funkce začnou vracet FINISHED.

## Třída Planner
Třída `Planner` se stará o vytváření plánů a komunikaci s uživatelem, což znamená naparsování a uložení plánů ze sériové komunikace a práci s EEPROM, ve které mohou být uložený již predzpracované plány. Parsování je navrhnuto obecně nad rozhraním podobné streamům, takže je případně možné přidat nové zdroje plánů. Proto třída dovoluje i přímo načítat plán z řetězce v kódu, což sloužilo spíše pro testování.

Celá třída je navržena jako statická, protože by stejně nemohla rozumně existovat vícekrát současně.

### Formát předzpracovaného plánu

`Planner` si udržuje dvě statická pole pro aktivní a rozparsovaný plán, celkem cca 400B.
To druhé pole je nutné k tomu, že pokud by parsování plánu někde uprostřed selhalo, tak aby aktivní plán zůstal nezměněn.

1. Struktura s výchozí konfigurací robota:
    - jeho poloha jako řádek a sloupec = 2*1B
    - orientace - enum(N,W,E,S) = 1B
2. počet instrukcí N<=64 = 1B
3. N 3B struktur reprezentující instrukce:
    - řádek, sloupec cíle - 2*4b = 1B
    - startovací čas, omezen na 6353,5 sekund = 15b
    - flag jestli se má nejdřív dosáhnout cílového řádku nebo sloupce = 1b

Maximální velikost celého plánu je 64*3B + 4B + 1B = 197B. Limit na 64 instrukcí na plán se dá změnit, ale limitující faktor je hlavně 1KB EEPROM, takže při zvětšení plánu je potřeba případně upravit počet slotů.

### Obsah EEPROM
1. Default slot = 1B
2. Hlavičky slotů - 5x struktura:
    - výchozí konfigurace robota = 4B
    - počet instrukcí = 1B
3. 5x pole délky \<Maximální délka plánu> s 3B stukturama reprezentujícíma instrukce.

Celkem je při 5 slotech s délkou plánů 64 zabráno 1B + 5*5B + 5*64*3B=986B EEPROM.

