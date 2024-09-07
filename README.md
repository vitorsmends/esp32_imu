
# esp32_imu

Este repositório contém o código para um sistema de medição inercial utilizando o sensor MPU6050 integrado ao ESP32, com transmissão de dados via Bluetooth. O sistema é capaz de capturar aceleração linear e velocidade angular em tempo real, e os dados podem ser processados no ROS2.

## Conexão com o ESP32 utilizando Ubuntu

Para conectar o ESP32 via Bluetooth, identifique o endereço MAC do dispositivo, que deve ser um código como `A0:A3:B3:89:25:A6`. Em seguida, execute o comando abaixo para associar o dispositivo a uma porta serial:

```sh
sudo rfcomm bind /dev/rfcomm0 A0:A3:B3:89:25:A6
```

## Dependências para utilizar a IMU

Instale a biblioteca MPU6050 no Arduino IDE ou no PlatformIO utilizando o link abaixo:

[MPU6050 Library](https://github.com/ElectronicCats/mpu6050)

## Resultados

O sistema desenvolvido foi testado e os dados de aceleração linear e velocidade angular foram capturados e visualizados, conforme mostrado nas figuras abaixo.

### Aceleração Linear

![Aceleração Linear](img/linear_acceleration.png)

### Velocidade Angular

![Velocidade Angular](img/angular_velocity.png)

## Utilizando o ROS2

Para processar os dados no ROS2, basta utilizar o arquivo de lançamento `imu.launch.py`. Este arquivo realiza a configuração necessária para a leitura dos dados recebidos via Bluetooth.

Execute o comando abaixo para iniciar o sistema no ROS2:

```sh
ros2 launch esp32_imu imu.launch.py
```

Este comando configurará os nós necessários para a captura e processamento dos dados de IMU no ROS2.

## Licença

Este projeto está sob a licença MIT. Consulte o arquivo LICENSE para mais informações.
