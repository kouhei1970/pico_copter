# pico_copter
Raspberry Pi Picoを用いたフライトコントローラー用飛行制御プログラムです。


### Raspberry Pi Pico
使用しているマイコンは安価なRaspberry Pi Picoです
いろいろなところで扱っていると思いますが、例えば秋月電子なら以下です。

https://akizukidenshi.com/catalog/g/gK-16149/

### LSM9DS1
使用している9DOFセンサのLSM9DS1についてはスイッチサイエンスのものであれば以下です。

https://www.switch-science.com/catalog/6451/

## Picoとセンサの接続

PicoとLSM9DS1との接続は以下の様にするものとします。

|Pico側|LSM9DS1ボード側|備考|
|---|---|---|
|GPIO 8 (pin 11) MISO/SPI0 RX|SDO（２ピンとも）|センサからデータ出力|
|GPIO 9 (pin 12) SPI00 CSn|CSM|地磁気計選択|
|GPIO 10 (pin 14) SCK/SPI0 SCK|SCL|クロック|
|GPIO 11 (pin 15) MOSI/SPI0 TX|SDA|センサへデータ出力|
|GPIO 13 (pin 17) SPI0 CSn|CSAG|加速度・ジャイロ選択|

### ブログ 
関連ブログを書いています

- https://rikei-tawamure.com/entry/2021/09/19/153553
- https://rikei-tawamure.com/entry/2021/09/27/111205
- https://rikei-tawamure.com/entry/2021/10/07/211725
- https://rikei-tawamure.com/entry/2021/08/28/172556
