# Bally OS - Robô ESP32-S3

Este projeto implementa o controle de um robô baseado em ESP32-S3, utilizando arquitetura orientada a objetos, FreeRTOS, comunicação ESP-NOW e execução paralela em dois núcleos.

> **Compatibilidade:**
> Este software está sendo desenvolvido para ser totalmente compatível com o hardware documentado no repositório [bally_robot](https://github.com/AlisonTristao/bally_robot).

> **Telemetria com T-Dongle S3 (LilyGO):**
> Para realizar a telemetria via ESP-NOW, é utilizado o T-Dongle S3 da LilyGO como receptor dos dados. O código desenvolvido para enviar comandos e logs do robô para o dongle está disponível em: [t_dongle_develop](https://github.com/AlisonTristao/t_dongle_develop).

## Estrutura do Projeto

```
├── include/       # Cabeçalho das definições globais necessárias para configurar o robô
├── lib/           # Bibliotecas principais (sensores, controle, motores, logger, etc.)
├── robot/         # Implementação dos estados da máquina de estados
├── src/           # Código principal (main.cpp)
├── utils/         # Objetos estáticos e utilitários globais
├── platformio.ini # Configuração do PlatformIO
```

### Principais Módulos

- **ArraySensor**: Gerencia o array de sensores frontais, incluindo calibração, leitura e normalização dos valores.
- **Encoder**: Gerencia a leitura dos encoders usando periféricos de hardware do ESP32.
- **Format**: Header-only, sem dependências — formatação de tamanhos em bytes (`"12.34 MB"`) reaproveitada por SDCard, USBMassStorage e Logger.
- **HBridge**: Controle dos motores via ponte H, incluindo direção e PWM.
- **BtpTransport**: Endpoint BTP v1 com identidade de boot, sequência atômica, codec, CRC e fragmentação comuns.
- **CommandProcessor**: Valida `COMMAND_REQUEST`, reserva uma chave de deduplicação por boot, executa cada intenção uma vez e reproduz o mesmo `COMMAND_RESULT` correlacionado nos retries.
- **ManifestResponder**: Responde `CONTROL/MANIFEST_REQUEST` descrevendo os dois schemas estáticos de `TelemetryPublisher` (`protocol.test`, `robot.state`) como `MANIFEST_DATA`; `config_revision` é uma constante (1), já que o catálogo deste firmware não muda em tempo de execução.
- **TxScheduler**: Filas estáticas separadas e FIFO por classe; transmite `COMMAND_RESULT > STATUS > LOG` crítico `> TELEMETRY > DEBUG`, com um único envio ESP-NOW pendente e contadores de aceite, entrega, timeout e drop.
- **Logger**: Ring de eventos em PSRAM; emite exclusivamente frames `LOG` BTP de tamanho real via ESP-NOW.
- **TelemetryPublisher**: Fila SPSC estática e não bloqueante para amostras `PACKED_LE`; publica `protocol.test` a 50 Hz e `robot.state` nas transições, preservando o timestamp da coleta.
- **OTAUpdater**: Atualização de firmware sem fio a partir do estado DEBUG — conecta a uma rede Wi-Fi cadastrada no cartão SD, anuncia `<hostname>.local` via mDNS e recebe o novo binário via HTTP (`POST /update`). `GET /status` (JSON: `device`, `online`, `firmware`, `ota_ready`) deixa uma ferramenta externa checar se o robô está pronto para receber o upload antes de mandá-lo.
- **RobotSettings**: Armazena e persiste (`settings.conf` no SD) todos os parâmetros configuráveis em runtime.
- **SDCard** / **USBMassStorage**: Acesso ao cartão SD e transferência de propriedade exclusiva do FAT entre o robô e um host USB.
- **StateMachine**: Máquina de estados do robô, com transições e *callbacks* configuráveis.
- **SystemMonitor**: Relatório opcional de saúde do sistema (CPU, memória, temperatura), habilitado por `ENABLE_SYSTEM_MONITOR`.
- **StaticObjects**: Inicializa e centraliza instâncias globais dos principais objetos (robô, sensores, motores, logger, etc.).
- **TinyShell**: Interpretador de linha de comando embarcado. Organiza comandos em módulos, suporta autocompletar (*auto-completion*), converte dinamicamente os tipos de argumentos de *strings* para os tipos esperados, valida a execução e lida com erros de forma segura (*try-catch*).

### Outras Pastas
- **robot/**: Implementação dos estados (Setup, Wait, Calibrate, Debug, Run, Finish, Telemetry, Error).
- **include/**: Definições globais necessárias para configurar o robô.
- **utils/**: Contém as configurações globais das funcionalidades do robô, centralizadas no objeto `ROBOT`, que permite criar apenas uma instância. Esse objeto unifica o acesso aos sensores, motores, utilidades dos sensores, logger, controle e demais recursos.

### Organização e Acoplamento entre Módulos

Cada pasta em `lib/` é compilável e compreensível isoladamente. Duas regras mantêm isso:

1. **Dependência entre bibliotecas só quando justificada, e sempre explícita.** A maioria dos módulos (`Flags`, `StateMachine`, `HBridge`, `Encoder`, `ArraySensor`, `SystemMonitor`, `Format`) não inclui nenhuma outra biblioteca do projeto. Onde uma dependência é real — `Logger` grava no `SDCard`; `OTAUpdater` usa `SDCard`/`Flags_out`; `USBMassStorage` usa `SDCard` — o header só faz `class Nome;` (forward declaration) e o `.cpp` inclui o header completo. Isso limita o acoplamento à implementação, não à interface pública: quem só usa a classe por referência/ponteiro nunca precisa saber o que ela inclui por baixo.
2. **`RobotSettings` não depende de nenhum outro módulo do projeto.** É a camada mais "de baixo" da configuração em runtime (dados + persistência em `settings.conf`); quem lê valores dela (`OTAUpdater`, `ArraySensor`, `Logger`, ...) depende dela, nunca o contrário. Os valores padrão compartilhados com o `OTAUpdater` (tempos, canal do ESP-NOW, ...) vivem em `lib/OTAUpdater/OtaDefaults.h`, um header sem nenhuma dependência que os dois incluem.

**Comandos de shell: cada módulo registra os próprios.** Toda classe que expõe comandos de shell implementa `register_shell_commands(TinyShell&, ...)` no seu próprio `.cpp`, recebendo por parâmetro só o que precisa (ex.: `OTAUpdater::register_shell_commands` recebe `Logger&`, `SDCard&`, `USBMassStorage&` e um `std::function<bool()>` para consultar se algum teste de DEBUG está ativo — sem precisar saber que "teste de DEBUG" é um conceito do `ROBOT`). `ROBOT::startWrappers()` (`utils/BallyRobot/BallyRobot.cpp`) é só a lista dessas chamadas de composição.

Três módulos de shell moram no próprio `ROBOT` (`registerRobotIOCommands`/`registerKalmanCommands`/`registerDebugCommands`, em `utils/BallyRobot/BallyRobot.cpp`), por não terem dono natural fora dele:
- **`robot`** (btn/ssr/set_pwm/set_led): aciona os `Flags_in`/`Flags_out`/`Flags_pwm` que o próprio `ROBOT` compõe; não há uma biblioteca "dona" além dele.
- **`kalman`** (estado do EKF + log periódico): o filtro (`TinyEKF`) é uma dependência externa vendorizada, sem wrapper próprio no `lib/`; quem possui a instância, o timer de amostragem e os vetores de controle/medição é o `ROBOT`.
- **`debug`** (`test_arr_sensor`/`test_encoder`): o agendamento e o *gate* (estado DEBUG + USB ocioso) são aplicados uniformemente sobre vários sensores a partir de estado privado do `ROBOT`, não pertencem a nenhum sensor individual.

`ROBOT`/`BallyRobot` é o *composition root*: o único lugar que conhece e instancia todos os subsistemas. Ele não contém a lógica de negócio de nenhum deles.

## Fluxo Detalhado do Sistema

O funcionamento do robô é dividido em duas partes principais, aproveitando os dois núcleos do ESP32-S3:

### Núcleo 1 (Core APP) — Execução Prioritária
O `void loop` roda exclusivamente no núcleo 1, especializado em executar a função principal de cada estado da máquina de estados. Isso garante que as funções críticas do robô tenham prioridade máxima, sem dividir processamento com tarefas paralelas.

### Núcleo 0 (Core PRO) — Processamento Paralelo
No segundo núcleo, é executada a rotina paralela do robô, responsável por:
- Verificar as *flags* dos botões, sensores laterais, sinais PWM e LEDs.
- Gerir as transições da máquina de estados, de acordo com as *flags*.
- Gerenciar a fila estática de comandos recebidos por ESP-NOW.
- Antes da fila, validar o frame BTP (magic, versão, tamanhos, CRC e fragmentação), o MAC/source da origem, o alvo do boot e o payload `COMMAND_REQUEST`. Somente a ação de shell `0x0001`, versão 1, chega ao **TinyShell**; `TELEMETRY`, `LOG`, `TERMINAL`, `CONTROL` e demais objetos são rejeitados.
- Reservar a identidade `(source_id, boot_id, sequence)` antes de executar; retries idênticos reutilizam o resultado armazenado, conflitos são rejeitados e fila cheia gera `BUSY/CAPACITY_EXHAUSTED` estruturado.
- Drenar o scheduler de transmissão sem deixar telemetria ou debug atrasarem resultados de comando; o callback ESP-NOW apenas confirma ou falha o único frame pendente.
- Gerenciar o Filtro de Kalman Estendido, fazendo a amostragem, o cálculo de predição e a atualização do estado. 

---

### Comunicação
- **ESP-NOW/BTP v1:** Utilizado para comunicação sem fio. `bally_protocol` 0.1.0 é integrado por dependência local identificável; nenhum `struct` C/C++ é transmitido.
- **Identidade:** `source_id` deriva do MAC de fábrica; `boot_id` é aleatório, não nulo e persistido em NVS para impedir repetição imediata. A sequência é única por mensagem lógica e compartilhada com segurança entre tasks.
- **Comandos:** o peer autorizado é o `MAC_ADDR` do build e seu `source_id` precisa corresponder ao MAC recebido. O payload da ação de shell é uma única linha de até 512 bytes, sem NUL, CR ou LF. O resultado BTP repete a tripla da requisição, `action_id/version`, status e erro; o cache estático de 16 entradas permanece durante todo o boot, portanto retry nunca repete o efeito.
- **Scheduler TX:** cada produtor entrega frames completos a uma fila bounded da sua classe. Há capacidade exclusiva para resultados de comando mesmo com 16 telemetrias pendentes. Telemetria e logs espontâneos não são retransmitidos; perda de resultado é recuperada pelo retry da requisição e replay do cache.

#### Logger (eventos e debug)
O Logger preserva o ring de PSRAM, mas o armazenamento interno não é formato de wire. Na inserção, cada evento recebe `timestamp_us` e uma sequência BTP. No `flush`, o payload é lido em blocos de até 210 bytes, codificado com CRC-32 pelo codec canônico e enviado em frames de tamanho exato. Arquivos `.blog` também contêm frames BTP concatenados e são validados antes do replay. Telemetria de alta frequência permanece fora do Logger e é publicada por `TelemetryPublisher`.

#### TelemetryPublisher (amostras binárias)

O publisher possui 16 slots pré-alocados e política explícita de `drop-newest`:
se a fila estiver cheia, a task de controle contabiliza e descarta a nova
amostra sem esperar. A sequência BTP e o `timestamp_us` são reservados na
coleta; a task `routine` apenas codifica e envia depois. Falhas imediatas de
rádio também são contabilizadas e removidas para não travar a fila.

Os schemas estáticos iniciais são `protocol.test` (`topic_id=0x0001`,
`counter:uint32`, `value:float32`) e `robot.state` (`topic_id=0x0002`,
`state:uint8`), ambos `PACKED_LE`, versão 1. Não há CSV, formatação textual,
terminador nem passagem pelo `Logger`.

### Flags (Eventos Temporizados e Segurança)
O projeto utiliza uma biblioteca de *flags* para facilitar a gestão de eventos temporizados e garantir segurança no controle dos atuadores:

- É possível definir um valor booleano (HIGH ou LOW) para cada *flag*, válido por um tempo determinado.
- No núcleo secundário, o sistema verifica periodicamente o tempo de cada *flag* e reseta automaticamente aquelas que expiraram.
- Isso permite notificar eventos disparados em interrupções (como botões ou sensores) de forma global.
- Para sinais de saída (*output*), também utilizamos *flags*: quando uma função deixa de enviar o sinal, a *flag* é desativada automaticamente, evitando que atuadores (motores, LEDs, etc.) fiquem ligados por tempo indeterminado.

### Máquina de Estados
O robô é controlado por uma máquina de estados robusta, com funcionamento definido em tempo de compilação:

- A máquina de estados possui um número fixo de estados, definidos em enumeração no código (ex: SETUP, WAIT, CALIBRATE, DEBUG, RUN, FINISH, TELEMETRY, ERROR).
- Cada estado possui:
  - Uma função principal (*main*) que é executada com prioridade máxima no loop principal (núcleo 1).
  - Uma função de verificação de transição, que avalia as condições (*flags*, sensores, comandos) para decidir se o estado deve mudar.
- Caso ocorra qualquer problema na gestão dos estados (ex: função não definida, erro de transição), a máquina de estados automaticamente coloca o sistema no estado ERROR e notifica o erro no logger, garantindo segurança e rastreabilidade.
- Tanto as funções principais quanto as de transição podem ser customizadas conforme a aplicação.

## Dependências externas

- **TinyShell** (adicionada via `lib_deps` no platformio.ini):
  Usado para a interpretação de comandos textuais de telemetria. Oferece:
  - Validação do módulo, nome do comando e número de argumentos.
  - Conversão dinâmica de *strings* com parâmetros de escape.
  - Execução de funções em tempo de execução do código.
- **TinyEKF** (adicionada via `lib_deps` no platformio.ini):
  Usado para estimar a velocidade linear e angular do robô, considerando uma dinâmica diferencial. Oferece:
  - Script em Python que otimiza o modelo não linear e desenrola o cálculo de multiplicações de matrizes.
  - Estimativa ótima dos estados, considerando a velocidade angular e linear.
  - Permite utilizar qualquer modelo; contudo, para este projeto, utilizamos acelerômetro, giroscópio e 2 encoders.

## Diagrama da Máquina de Estados

![Fluxo da Máquina de Estados](docs/StateMachine_flux.png)

---
Projeto desenvolvido por Alison Tristão.
