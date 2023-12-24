# Инструкция по подключению умного чайника к сервису Dealgate через Яндекс:
В этом репозитории храниться проект по созданию чайника, управляемого через интернет.
1. Регистрация аккаунта:
  * Перейдите по ссылке (https://dealgate.ru/) и зарегистрируйте аккаунт, используя учетную запись Яндекс.
2. Настройка профиля:
  * Перейдите в раздел "Профиль".
  * Заполните данные MQTT-сервера в полях "username" и "password". Запомните созданные данные.
  ![](https://github.com/SSONHOK/Teapot/blob/main/lib/1.png)
3. Подключение платы:
  * Перейдите к настройкам платы и подключите её согласно схеме.
  ![](https://github.com/SSONHOK/Teapot/blob/main/lib/2.png)
4. Изменение данных в коде:

  * Скачайте sketch-файл с названием "Teapot.ino".
  * Замените в коде значения "ssid" и "pass" на ваши данные для авторизации.
5. Прошивка и монитор порта:

  * Прошейте плату.
  * Откройте монитор порта и проверьте id устройства, который выводится каждую секунду.
6. Регистрация устройства на сайте:

  * Скопируйте id устройства после "Sending message to topic".
  * Вернитесь на сайт (https://dealgate.ru/) и перейдите в "Меню устройства".
  * Нажмите "Новое устройство" и вставьте id в поле "MQTT Client id".
  * Заполните остальные данные по устройству (по желанию) и сохраните устройство.
7. Настройка умений:

  * В списке устройств выберите нужное устройство.
  * Нажмите на шестерёнку рядом с устройством.
  * В разделе "Умения" нажмите "+ Датчик".
8. Настройка датчика:

  * Выберите тип датчика — термометр.
  * Введите в поле MQTT topic значение, полученное из монитора порта.
  * Включите оповещение Яндекс, выберите единицы измерения — "градус Цельсия".
  * Задайте наименование умения и сохраните его.
  ![](https://github.com/SSONHOK/Teapot/blob/main/lib/3.png)

9. Подключение к приложению "Умный дом":

  * Откройте приложение "Умный дом".
  * Нажмите на "+" в верхнем правом углу.
  * Выберите "Устройства умного дома".
  * Найдите в списке навыков "Dealgate" и выполните шаги 7 и 8 на странице инструкции.
Теперь ваше устройство должно успешно интегрироваться с сервисом Dealgate через Яндекс.
