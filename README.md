# kydas_driver

Pacote de comunicação para _ROS noetic_ entre motores/drivers da empresa KEYA para o modelo de driver KYDAS41601A.

## Nodes

### kydas_driver

Permite a comunicação via serial para o driver KYDAS41601A.

#### Subscribed Topics

- cmd_speed (kydas__driver/CmdSpeed)
	Setar velocidade do motor, o dado tem que estar em um intervalo de [-10000, 10000]

#### Published Topics

- speed (kydas_driver/MotorSpeed)
	Velocidade do motor, no membro `speed` a velocidade está em graus/s , e em `rawSpeed` está o dado não tratado enviado pelo driver

- position (kydas_driver/MotorPosition)
	Posição do motor, no membro `position` a posição está em graus no intervalo [-360, 360] , e em `rawPosition` está o dado não tratado enviado pelo driver

- rotorPosition (kydas_driver/MotorRotorPosition)
	Dado sobre a posição do rotor

- eletricAngle (kydas_driver/MotorEletricAngle)
	Dado sobre angulo elétrico do motor

- current (kydas_driver/MotorCurrent)
	Corrente que passa pelo motor, em Amps

- voltage (kydas_driver/MotorVoltage)
	Tensão que passa pelo motor, em Volts

- temperature (sensor_msgs/Temperature)
	Temperatura media pelo motor, em °C

- controllerStatus (kydas_driver/MotorControllerStatus)
	Condição do motor, o membro `controlMode` indica como o driver está sendo controlado, `feedbackWay` a maneira que as medições de posição está sendo feitas e `workingMode` a maneira com que se está controlando a velocidade/posição/torque do motor (vide manual do driver para mais informações na página 25) 

- faultCode (kydas_driver/MotorFaultCode)
	Código de falha, para se entender o que ele significa é necessário visualizar o dado em binário, informações de como interpretar estão no manual do driver na página 26

- programVersion (kydas_driver/MotorProgramVersion)
	Versão do programa
	
### Parameters

- ~port (int, default: 16) 
	Port em que se encontra o conector serial-usb do driver

- ~bdrate (int, default: 115200)
	O Baudrate da comunicação serial que será usada (apenas altere esse valor se realmente necessário)
	
- ~loop_rate (float, default: 25)
	A frequencia em que o loop de enviar/receber/processar informações irá funcionar, valores pequenos irão criar uma maior demora entre envio do comando e resposta do driver, enquanto valores maiores podem fazer com que mensagens não sejam lidas pelo driver
	
- ~request__data_rate (float, default: 8)
	A frequencia em que o loop de pedir informações ao driver irá rodar, ele é necessário para que o driver saiba que ainda existe comunicação com ele, portanto valores abaixo de 2 podem fazer com que ele desligue sozinho, já valores muito altos podem fazer com que comandos sejam atrasados pelo volume de informações
	
- ~response__check_time (float, default: 0.25)
	O periodo em que o loop de verificar se o driver ainda está conectado roda
	
- ~timeout_time (float, default:2)
	O tempo que esperamos por uma resposta do driver até decidir que ele não está mais conectado
