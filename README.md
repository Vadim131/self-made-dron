# self-made-dron
Привет всем!:)) Это мой текущий проект, над которым я работаю. На текущий момент я реализую ИНС. Пишу код на stm32f3discovery. К сожалению, встроенные на плату датчики плохо показали себя в работе, поэтому было решено использовать mpu-6050. В скором времени планирую опробовать свои наработки на практике, установив плату на макет с двигателями.

Сейчас я уже реализовал базовые функции, необходимые для работы мк (Необходимое тактирование, включил FPU-модуль, DWT, настроил прерывания и т.д.), а также настроил I2C, USART и пообщался с компьютером через него. Мне удалось получить данные с гироскопа и акселерометра и буквально день назад получилось рассчитать углы положения платы в пространстве) Я планирую сделать управление через интернет, соотв. для этих целей я выбрал плату gsm-модуля sim800l. Ее я тоже успел опробовать в работе и в дальнейшем припаяю к stm-ке

Фотографии прилагаются
1) USART  общение с sim800l
![](https://github.com/Vadim131/self-made-dron/raw/main/Images/sim800l_usart.jpeg)

2) USART c stm32f3discovery
![](https://github.com/Vadim131/self-made-dron/raw/main/Images/stm_usart.jpeg)

Фильтр Калмана заработал. Ура! 
12.07.2024: Работаю над системой стабилизации и PID. Осуществил модель тестовой демонстрации разработанной системы. Фото ниже
![](https://github.com/Vadim131/self-made-dron/raw/main/Images/stm_test_imu.jpeg)
