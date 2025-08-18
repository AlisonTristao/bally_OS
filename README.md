# Como o código está organizado? 

A estrutura e organização do código é majoritariamente baseada em orientação a objetos.  
Assim, todas as funcionalidades estão divididas em classes, seguindo a estrutura a seguir:

## 📂 include
Possui todos os cabeçalhos do código (funcionalidades e definições simples o suficiente para não precisarem de orientação a objetos).
- **EspNow**  
  Cabeçalho responsável por organizar as funções relacionadas à comunicação por radiofrequência EspNow.
- **ParallelProcessing**  
  Contém todo o código que irá rodar no segundo núcleo, em paralelo com o `void loop`.
- **Pinout**  
  Define todos os pinos do robô.
- **Settings**  
  Cabeçalho que contém as configurações: sentido dos pinos, definições de tempo, etc.
- **Wrapper**  
  Responsável por integrar todas as bibliotecas, relacionado às funções que possuem dependência entre si.  
  Além disso, contém o empacotamento das funções para controle usando o CMD (TinyShell).

## 📂 lib
Possui todas as bibliotecas com os objetos utilizados no robô.
- **ArraySensor**  
  Objeto para gerenciar o sensor frontal.
- **Control**  
  Contém as funções do controle utilizado.
- **Encoder**  
  Código para gerenciar os encoders.
- **HBridge**  
  Biblioteca responsável pelo controle dos motores.
- **TinyShell**  
  Gerencia os comandos recebidos em forma de string, executando funções configuráveis.

## 📂 robot
Contém o código de todos os estados da máquina de estados.

## 📂 src
Raiz do código, que contém o algoritmo principal a ser executado. 

## 📂 utils
Possui as bibliotecas das classes estáticas do código.
- **Flags**  
  Responsável por calcular o tempo de passagem de variáveis que marcam o tempo de eventos (botões, sensores laterais e LEDs).
- **Logger**  
  Salva todos os registros do código e também recebe comandos em forma de texto.
- **StateMachine**  
  Gerencia o fluxo da máquina de estados do robô, utilizando os botões.
- **StaticObjects**  
  Inicializa os objetos utilizados, para uso em todos os outros arquivos.

# Como o código funciona? 

O fluxo do programa é baseado em uma **máquina de estados**.  
A função principal roda dedicada no núcleo primário (core 1) através do `void loop`, enquanto a gestão do fluxo da máquina de estados, junto de outras funcionalidades (interrupções, etc.), roda no segundo núcleo.  

Essa organização garante que as funcionalidades críticas sejam executadas com **período fixo**, enquanto funções com tempo incerto e de menor prioridade são processadas em paralelo.

# Fluxo da Máquina de Estados  

![Fluxo da Máquina de Estados](StateMachine_flux.png)
