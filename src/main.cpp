#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <microDS18B20.h>

//Настройка пинов
#define HEATERPIN   D0                      //Пин к которому подключено реле для включения нагревателя
#define DS18        D1                      //Пин к которому подключен термистор

//Объявлнение датчика
MicroDS18B20<DS18> sensor;

//Настройка сети
#define SSID        "Red8"             // Название сети
#define PASS        "244466666"             // Пароль для сети

//Настройка MQTT
#define USERNAME    "Labalava"              // username указанный в dealgate
#define PASSWORD    "qwerty"              // password указанный в dealgate

#define BROCKER     "mqtt.dealgate.ru"      //MQTT брокер
#define PORT        1883                    //Порт для MQTT-соединения
#define KEEP_ALIVE  50000                   //вемя допустимое для отсутствия соединения в милисекундах

//Настройка параметров времени
#define INTERVAL    1000                    //Интервал отправки данных в MQTT в миллисекундах

unsigned long previousMillis = 0;           //Время предыдущей отправки данных в MQTT в миллисек       

void onMqttMessage(int messageSize);        //Прототип функции обработки сообщений MQTT

byte mac[6];                                // MAC-адрес, он же, в шеснадцатиричном представлении, идентификатор устройства
char macStr[12];                            // Текстовое представление MAC-адреса


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);          // Объект MQTT-соединения


//Топики
String      TemperatureTopic;               //Топик для отпраки данных об актуальной температуре
String      WantedTempTopic;                //Топик для получения данных о желаемой температуре
String      OnOfTopic;                      //Топик для получения данных о состоянии работы
String      ModeTopic;                      //Топик для получния данных о режиме работы

uint        WantedTemp = 90;                //Желаемая температура в градусах цельсия
bool        onOff = false;                  //Состояние работы
bool        mode = false;                   //Режим работы false - нагрев, true - поддержание температуры
uint        temperature;                    //Текущая температура в градусах цельсия

void setup() 
{
    //Активируем наш отладочный светодиод
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(HEATERPIN, OUTPUT);
    sensor.requestTemp();                   //Сразу опрашиваем температуру, чтобы вывести значение когда войдём в loop

    //Открываем порт и начинаем писать даные, чтобы их можно было залогировать
    //Сочетание символов "#t#" будем использовать в качестве метки времени при записи логов, заменяя "#t#" на время получния сообщения
    Serial.begin(9600);
    Serial.println("#t# - Board starded working");

    //Открываем соединение с сетью
    Serial.println("#t# - Connecting to the network...");
    while (WiFi.begin(SSID, PASS) != WL_CONNECTED) 
    {
        // failed, retry
        Serial.print(".");
        delay(500);
    }
    Serial.println("#t# - You are connected to the network");
    
    //Получаем MAC-адрес и генерируем его текстовое представление
    Serial.print("#t# - Yor MAC-address is: ");
    WiFi.macAddress(mac);
    for (int i = 0; i < 12; i+=2) 
    {
        char buf[2];
        sprintf(buf, "%02x", mac[i/2]);
        macStr[i] = buf[0];
        macStr[i+1] = buf[1];
    }
    Serial.println(macStr);

    //Генерируем топики для MQTT
    //Будем использовать формат топика: "MAC/Topic"
    TemperatureTopic = macStr;
    WantedTempTopic = macStr;
    OnOfTopic = macStr;
    ModeTopic = macStr;
    TemperatureTopic += "/Temperature";
    WantedTempTopic += "/WantedTemp";
    OnOfTopic += "/OnOf";
    ModeTopic += "/Mode";

    //Открываем соединение с MQTT-соединением
    Serial.print("#t# - Attempting to connect to the MQTT broker: ");
    Serial.println(BROCKER);
    mqttClient.setKeepAliveInterval(KEEP_ALIVE);
    mqttClient.setId(macStr);
    mqttClient.setUsernamePassword(USERNAME, PASSWORD);
    if (!mqttClient.connect(BROCKER, PORT)) 
    {
        Serial.print("#t# - MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        rp2040.reboot();
    }
    Serial.println("#t# - MQTT connection successful");

    //Подписываемся на топик для получения данных о включении или выключении насоса
    mqttClient.onMessage(onMqttMessage);//Передаём функцию обработки сообщений MQTT в обработчик
    Serial.print("#t# - Subscribing to topic: ");
    Serial.println(OnOfTopic);
    mqttClient.subscribe(OnOfTopic);
    Serial.print("#t# - Subscribing to topic: ");
    Serial.println(ModeTopic);
    mqttClient.subscribe(ModeTopic);
    Serial.print("#t# - Subscribing to topic: ");
    Serial.println(WantedTempTopic);
    mqttClient.subscribe(WantedTempTopic);

    //Завершение инициализации
    Serial.println("#t# - Controller initialized");
    Serial.println();
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{
    mqttClient.poll();  //Опрашиваем MQTT

    //Блок проверяющий соединение с брокером и WiFi, при отсутствии соединения перезагружаемся
    if (!wifiClient.connected() or !mqttClient.connected())
    {
        rp2040.reboot();
    }

    if (millis() - previousMillis >= INTERVAL)
    {
        digitalWrite(LED_BUILTIN, HIGH);        //Активируем светодиод для индикации отправки данных в MQTT

        sensor.requestTemp();     

        temperature = sensor.getTemp();    //Чтиаем показатели термистора

        //Выводим данные в порт
        Serial.print("#t# - Temperature: ");
        Serial.println(temperature);

        //Отправляем данные о температуре в топик MQTT
        mqttClient.beginMessage(TemperatureTopic);
        mqttClient.print(temperature);
        mqttClient.endMessage();

        Serial.print("#t# - Data published to topic: ");
        Serial.println(TemperatureTopic);

        //Заканчиваем отправку
        Serial.println();
        previousMillis = millis();      //Обновляем таймер
        digitalWrite(LED_BUILTIN, LOW); //Гасим светодиод по окончании отправки данных в MQTT
    }

    if (mode == true) 
    //Обрабатываем поддержание температуры
    {
        if (onOff == true || temperature > WantedTemp)  
        {
            if (temperature >= WantedTemp) 
            {
                digitalWrite(HEATERPIN, LOW);
            }
            else 
            {
                digitalWrite(HEATERPIN, HIGH);
            }
        }
    }
    else
    //Обрабатываем нагревание 
    {   
        if (onOff == true)
        {
            if (temperature >= WantedTemp) 
            {
                digitalWrite(HEATERPIN, LOW);
                onOff = false;
            }
        } 
    }
}


void onMqttMessage(int messageSize)
{
    digitalWrite(LED_BUILTIN, HIGH); //Снова включаем наш волшебный светодиод

    //Вывод данных о сообщении в порт
    Serial.print("#t# - Message received from topic: ");
    String topic = mqttClient.messageTopic();
    Serial.print(topic);
    Serial.print(", length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    //Обрабатываем сообщение
    String message = "";
    for (int i = 0; i < messageSize; i++) 
    {
        message += (char)mqttClient.read();
    }
    Serial.print("#t# - Message: ");
    Serial.println(message);

    //Проверяем топик
    if (topic == ModeTopic)
    //Просто включаем или выключаем помпу
    {
        if (message == "heat")
        {
            Serial.println("#t# - Heating mode on");
            mode = false;
        }
        else if (message == "keep")
        {
            Serial.println("#t# - keep temperature mode on");
            mode = true;
        }
    }
    if (topic == OnOfTopic)
    {
        if (message == "on")
        {
            Serial.println("#t# - Heating on");
            onOff = true;
        }
        else if (message == "off")
        {
            Serial.println("#t# - Heating off");
            onOff = false;
        }
    }
    if (topic == WantedTempTopic)
    {
        Serial.print("#t# - Wanted temperature: ");
        Serial.println(message);
        WantedTemp = message.toInt();
    }
    Serial.println();
    digitalWrite(LED_BUILTIN, LOW); //Гасим лапочку
}
