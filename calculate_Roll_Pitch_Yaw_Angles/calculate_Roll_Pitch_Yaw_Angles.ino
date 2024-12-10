// bu kodta ivmeölçer kalibrasyonu yaptım sonra manyetometre sürekli kalibrasyon yaptım 
#include <Wire.h>

// MPU6050 ve QMC5883L için I2C adresleri
#define MPU6050_ADDR 0x68
#define QMC5883L_ADDR 0x0D

// Kalibrasyon için sınır değerler

int16_t minMagX = 32767, maxMagX = -32768;
int16_t minMagY = 32767, maxMagY = -32768;
int16_t minMagZ = 32767, maxMagZ = -32768;

float scaleFactorCorrectionAccelX, scaleFactorCorrectionAccelY, scaleFactorCorrectionAccelZ;
float offsetAccelX, offsetAccelY, offsetAccelZ;

// Pitch ve Roll için değişkenler
float pitch; // static
float roll;
float yaw;

// Filtrelenmiş için değişkenler
float filtredMagX, filtredMagY, filtredMagZ;
float filtredAccelX, filtredAccelY, filtredAccelZ;

// Zamanlama
uint32_t LoopTimer;

// Medyan filtresi için Buffer(geçici depo)
float magXBuffer[5] = { 0, 0, 0, 0, 0 };
float magYBuffer[5] = { 0, 0, 0, 0, 0 };
float magZBuffer[5] = { 0, 0, 0, 0, 0 };

float accelXBuffer[5] = { 0, 0, 0, 0, 0 };
float accelYBuffer[5] = { 0, 0, 0, 0, 0 };
float accelZBuffer[5] = { 0, 0, 0, 0, 0 };

void CalibrationAccel(float *scaleFactorCorrectionX, float *scaleFactorCorrectionY, float *scaleFactorCorrectionZ,
                      float *offsetX, float *offsetY, float *offsetZ) {

  int16_t minAccelX = 32767, maxAccelX = -32768;
  int16_t minAccelY = 32767, maxAccelY = -32768;
  int16_t minAccelZ = 32767, maxAccelZ = -32768;

  int16_t rawX, rawY, rawZ;
  LoopTimer = millis();

  Serial.println("ivmeölçer +X yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  // Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawX > maxAccelX) maxAccelX = rawX;
    if (rawX < minAccelX) minAccelX = rawX;

    delay(1);
  }
  LoopTimer = millis();

  Serial.println("ivmeölçer -X yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  //Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawX > maxAccelX) maxAccelX = rawX;
    if (rawX < minAccelX) minAccelX = rawX;

    delay(1);
  }
  LoopTimer = millis();

  Serial.println("ivmeölçer +Y yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  //Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawY > maxAccelY) maxAccelY = rawY;
    if (rawY < minAccelY) minAccelY = rawY;

    delay(1);
  }
  LoopTimer = millis();

  Serial.println("ivmeölçer -Y yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  //Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawY > maxAccelY) maxAccelY = rawY;
    if (rawY < minAccelY) minAccelY = rawY;


    delay(1);
  }
  LoopTimer = millis();

  Serial.println("ivmeölçer +Z yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  //Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawZ > maxAccelZ) maxAccelZ = rawZ;
    if (rawZ < minAccelZ) minAccelZ = rawZ;

    delay(1);
  }
  LoopTimer = millis();

  Serial.println("ivmeölçer -Z yönünü hazırla...");
  while (millis() - LoopTimer < 4000)
    ;  //Bekle
  Serial.println("kalibrasyon başladı");
  for (int i = 0; i < 4000; i++) {
    readMPU6050(&rawX, &rawY, &rawZ);
    if (rawZ > maxAccelZ) maxAccelZ = rawZ;
    if (rawZ < minAccelZ) minAccelZ = rawZ;

    delay(1);
  }


  // Offset hesapla (Hard Iron Düzeltmesi)
  *offsetX = (maxAccelX + minAccelX) / 2.0;
  *offsetY = (maxAccelY + minAccelY) / 2.0;
  *offsetZ = (maxAccelZ + minAccelZ) / 2.0;

  // Ölçek hesapla (Soft Iron Düzeltmesi)
  float scaleX = (maxAccelX - minAccelX) / 2.0;
  float scaleY = (maxAccelY - minAccelY) / 2.0;
  float scaleZ = (maxAccelZ - minAccelZ) / 2.0;
  float avgScale = (scaleX + scaleY + scaleZ) / 3.0;

  // scale Vector düzeltmesi değerleri hesapla
  *scaleFactorCorrectionX = avgScale / scaleX;
  *scaleFactorCorrectionY = avgScale / scaleY;
  *scaleFactorCorrectionZ = avgScale / scaleZ;
}

// Kalibre edilmiş ivmeölçer değerlerini hesapla
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ,
                        float *calibratedAccelX, float *calibratedAccelY, float *calibratedAccelZ) {

  // Kalibre edilmiş değerleri hesapla
  *calibratedAccelX = (rawAccelX - offsetAccelX) * scaleFactorCorrectionAccelX;
  *calibratedAccelY = (rawAccelY - offsetAccelY) * scaleFactorCorrectionAccelY;
  *calibratedAccelZ = (rawAccelZ - offsetAccelZ) * scaleFactorCorrectionAccelZ;
}



//  manyetometre kalibrasyon verilerini güncelle
void updateCalibrationMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ, int16_t &minMagX, int16_t &maxMagX, int16_t &minMagY, int16_t &maxMagY, int16_t &minMagZ, int16_t &maxMagZ) {
  if (rawMagX < minMagX) minMagX = rawMagX;
  if (rawMagX > maxMagX) maxMagX = rawMagX;
  if (rawMagY < minMagY) minMagY = rawMagY;
  if (rawMagY > maxMagY) maxMagY = rawMagY;
  if (rawMagZ < minMagZ) minMagZ = rawMagZ;
  if (rawMagZ > maxMagZ) maxMagZ = rawMagZ;
}

// Kalibre edilmiş Manyetometre değerlerini hesapla
void getCalibratedMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ, int16_t minMagX, int16_t maxMagX, int16_t minMagY, int16_t maxMagY, int16_t minMagZ, int16_t maxMagZ, float *calibratedMagX, float *calibratedMagY, float *calibratedMagZ) {
  // Offset hesapla (Hard Iron Düzeltmesi)
  float offsetMagX = (maxMagX + minMagX) / 2.0;
  float offsetMagY = (maxMagY + minMagY) / 2.0;
  float offsetMagZ = (maxMagZ + minMagZ) / 2.0;

  // Ölçek hesapla (Soft Iron Düzeltmesi)
  float scaleX = (maxMagX - minMagX) / 2.0;
  float scaleY = (maxMagY - minMagY) / 2.0;
  float scaleZ = (maxMagZ - minMagZ) / 2.0;
  float avgScale = (scaleX + scaleY + scaleZ) / 3.0;

  // Kalibre edilmiş değerleri hesapla
  *calibratedMagX = (rawMagX - offsetMagX) * (avgScale / scaleX);
  *calibratedMagY = (rawMagY - offsetMagY) * (avgScale / scaleY);
  *calibratedMagZ = (rawMagZ - offsetMagZ) * (avgScale / scaleZ);
}

// MPU6050 başlatma
void initMPU6050(void) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Uyku modundan çık
  Wire.write(0x00);
  Wire.endTransmission();
}

// MPU6050'dan İvmeölçer verilerini oku
void readMPU6050(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
  Wire.beginTransmission(MPU6050_ADDR);  // bu dört satırı alçak geçiş filtresini aç
  Wire.write(0x1A);
  Wire.write(0x05);  //5 byte yaz
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);  //burada 8g'yi ayarlamak (çözünürlüğü ayarlamak)
  Wire.write(0x1C);                      //ivmeölçer çıkışlarını 1C adresinde(register) saklanır bu adresten gelen ölçüm aralığı AFS_SEL'e (JİROSKOP GÜRÜLTÜ PERFORMANSI) göre 2 yani 10 yani 16 bit aralığ o da bizim sonraki satır tanımı
  Wire.write(0x10);                      //0x10 adresi iki tabanlıda gösterimi 0001 0000 o da 16 bit eder //
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);  //ivmeölçer değerlerinden çekmesini başlamak
  Wire.write(0x3B);                      // ivmeölçer değerleri ilk saklanan değerin adresi AccX
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // bu fonkisyon 0x68 dresten iletilen verilerinden 6 baytı talep edeceğiz -- Normalde biz Master olarak biz veri göndeririz ama bu fonkisyonda Slave'i Master yaptık ve Master'ı Slave yaptık

  if (Wire.available() == 6) {
    *accelX = Wire.read() << 8 | Wire.read();  //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
    *accelY = Wire.read() << 8 | Wire.read();
    *accelZ = Wire.read() << 8 | Wire.read();
  }
}
// QMC5883L Sensörünü Başlat
void initQMC5883L() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);  // Continuous mode, 200Hz output, 8G range, 512 OSR
  Wire.endTransmission();
}

// QMC5883L manyetometre verilerini oku
void readQMC5883L(int16_t *magX, int16_t *magY, int16_t *magZ) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);  // Veri okuma başlangıç adresi
  Wire.endTransmission();
  Wire.requestFrom(0x0D, 6);

  if (Wire.available() == 6) {
    *magX = Wire.read() | (Wire.read() << 8);
    *magY = Wire.read() | (Wire.read() << 8);
    *magZ = Wire.read() | (Wire.read() << 8);
  }
}

// Medyan filtresi
float applyMedianFilter(float newData, float buffer[5]) {
  // 1. Yeni veriyi ekle (en eski veriyi güncelle)
  for (int i = 4; i > 0; i--) {
    buffer[i] = buffer[i - 1];
  }
  buffer[0] = newData;

  // 2. Dizi elemanlarını sıralamak için geçici bir kopya oluştur
  float sortedBuffer[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  for (int i = 0; i < 5; i++) {
    sortedBuffer[i] = buffer[i];
  }

  // 3. Sıralama
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (sortedBuffer[j] > sortedBuffer[j + 1]) {
        float temp = sortedBuffer[j];
        sortedBuffer[j] = sortedBuffer[j + 1];
        sortedBuffer[j + 1] = temp;
      }
    }
  }

  if (isnan(sortedBuffer[2])) {
    Serial.println("NaN tespit edildi!");
    return 0.0;  // Hata durumunda varsayılan değer döndür
  } else {
    // 4. Ortanca (medyan) değeri döndür
    return sortedBuffer[2];
  }
}

// Low Pass filtresi uygulamak
float applyLowPassFilter(float currentValue, float newValue, float alpha) {
  return alpha * newValue + (1 - alpha) * currentValue;
}

// Tilt Compensation ve Yaw hesaplama
void calculateYaw(float filtredAccelX, float filtredAccelY, float filtredAccelZ, float filtredMagX, float filtredMagY, float filtredMagZ) {
  /// Pitch ve Roll açılarını hesapla (Radyan)
  pitch = atan2(-filtredAccelX, sqrt(filtredAccelY * filtredAccelY + filtredAccelZ * filtredAccelZ));  //radyan cinsinden
  roll = atan2(filtredAccelY, sqrt(filtredAccelX * filtredAccelX + filtredAccelZ * filtredAccelZ));

  //Tilt Combinsation (Eğim telafisi hesapla)
  float Xm = filtredMagX * cos(pitch) + filtredMagZ * sin(pitch);
  float Ym = filtredMagX * sin(roll) * sin(pitch) + filtredMagY * cos(roll) - filtredMagZ * sin(roll) * cos(pitch);

  // X ve Y eksenlerinden Yaw (heading/azimut) açısını hesapla
  yaw = atan2(Ym, Xm) * 180.0 / 3.141592654;
  if (yaw < 0) yaw += 360;  // 0-360 derece aralığına taşı
}


void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);  // I2C hızını ayarla
  Wire.begin();
  delay(250);

  initMPU6050();  // MPU6050 Sensörünü Başlat
  CalibrationAccel(&scaleFactorCorrectionAccelX, &scaleFactorCorrectionAccelY, &scaleFactorCorrectionAccelZ,
                   &offsetAccelX, &offsetAccelY, &offsetAccelZ);

  initQMC5883L();  // QMC5883L Sensörünü Başlat
  Serial.println("Kalibrasyon için manyomeytre sensörü farklı yönlerde hareket ett...");
  LoopTimer = micros();
}




void loop() {

  int16_t rawAccelX, rawAccelY, rawAccelZ;                     // Ham ivmeölçer verileri
  float calibratedAccelX, calibratedAccelY, calibratedAccelZ;  // Kalibre edilmiş ivmeölçer verileri

  int16_t rawMagX, rawMagY, rawMagZ;                     // Ham manyetometre verileri
  float calibratedMagX, calibratedMagY, calibratedMagZ;  // Kalibre edilmiş manyetometre verileri

  // MPU6050 verilerini oku
  readMPU6050(&rawAccelX, &rawAccelY, &rawAccelZ);
  getCalibratedAccel(rawAccelX, rawAccelY, rawAccelZ,
                     &calibratedAccelX, &calibratedAccelY, &calibratedAccelZ);

  // QMC5883L manyetometre verilerini oku
  readQMC5883L(&rawMagX, &rawMagY, &rawMagZ);

  // Kalibrasyon verilerini güncelle
  updateCalibrationMag(rawMagX, rawMagY, rawMagZ, minMagX, maxMagX, minMagY, maxMagY, minMagZ, maxMagZ);

  // Kalibre edilmiş değerleri hesapla
  getCalibratedMag(rawMagX, rawMagY, rawMagZ, minMagX, maxMagX, minMagY, maxMagY, minMagZ, maxMagZ, &calibratedMagX, &calibratedMagY, &calibratedMagZ);

  // Medyan filtresini uygula
  float medianAccelX = applyMedianFilter(calibratedAccelX, accelXBuffer);
  float medianAccelY = applyMedianFilter(calibratedAccelY, accelYBuffer);
  float medianAccelZ = applyMedianFilter(calibratedAccelZ, accelZBuffer);

  float medianMagX = applyMedianFilter(calibratedMagX, magXBuffer);
  float medianMagY = applyMedianFilter(calibratedMagY, magYBuffer);
  float medianMagZ = applyMedianFilter(calibratedMagZ, magZBuffer);

  //Low Pass filtresini uygula
  filtredAccelX = applyLowPassFilter(filtredAccelX, medianAccelX, 0.10);
  filtredAccelY = applyLowPassFilter(filtredAccelY, medianAccelY, 0.10);
  filtredAccelZ = applyLowPassFilter(filtredAccelZ, medianAccelZ, 0.10);

  filtredMagX = applyLowPassFilter(filtredMagX, medianMagX, 0.10);
  filtredMagY = applyLowPassFilter(filtredMagY, medianMagY, 0.10);
  filtredMagZ = applyLowPassFilter(filtredMagZ, medianMagZ, 0.10);

  // Yaw/heading/ baş açısını hesapla
   filtredAccelX, filtredAccelY, filtredAccelZ, filtredMagX, filtredMagY, filtredMagZ);





  // Seri porttan verileri yazdır

  // Serial.print(" rawAccelX: ");
  //Serial.print(rawAccelX);
  //Serial.print(" rawAccelY: ");
  //Serial.print(rawAccelY);
  //Serial.print(" rawAccelZ: ");
  //Serial.print(rawAccelZ);

  //Serial.print(" calibratedAccelX: ");
  //Serial.print(calibratedAccelX);
  //Serial.print(" calibratedAccelY: ");
  //Serial.print(calibratedAccelY);
  //Serial.print(" calibratedAccelZ: ");
  //Serial.print(calibratedAccelZ);

  //Serial.print(" medianAccelX: ");
  //Serial.print(medianAccelX);
  //Serial.print(" medianAccelY: ");
  //Serial.print(medianAccelY);
  //Serial.print(" medianAccelZ: ");
  //Serial.print(medianAccelZ);

  //Serial.print(" medianMagX: ");
  //Serial.print(medianMagX);
  //Serial.print(" medianMagY: ");
  //Serial.print(medianMagY);
  //Serial.print(" medianMagZ: ");
  //Serial.print(medianMagZ);

  Serial.print(" Roll: ");
  Serial.print(roll * 180.0 / 3.141592654);
  Serial.print("°");
  Serial.print(" Pitch: ");
  Serial.print(pitch * 180.0 / 3.141592654);
  Serial.print("°");
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.println("°");

  // Döngü zamanlaması
  while (micros() - LoopTimer < 1000)
    ;  //Nan statment
  LoopTimer = micros();
}
