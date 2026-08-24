# Bally_OS - Firmware ESP32-S3

Este projeto implementa o controle de um robô baseado em ESP32-S3, utilizando arquitetura orientada a objetos, FreeRTOS, comunicação ESP-NOW e execução paralela em dois núcleos. A telemetria e os comandos trafegam via [BTP](https://github.com/AlisonTristao/BTP), um protocolo binário de comunicação e plotagem de dados em tempo real sobre ESP-NOW.

> **Compatibilidade:**
> Este software está sendo desenvolvido para ser totalmente compatível com o hardware documentado no repositório [bally_robot](https://github.com/AlisonTristao/bally_robot).

> **Telemetria com T-Dongle S3 (LilyGO):**
> Para realizar a telemetria via ESP-NOW, é utilizado o T-Dongle S3 da LilyGO como receptor dos dados. O código desenvolvido para enviar comandos e logs do robô para o dongle está disponível em: [Bally_dongle](https://github.com/AlisonTristao/Bally_dongle).

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
- **RobotSettings**: Armazena e persiste (`settings.conf` no SD) todos os parâmetros configuráveis em runtime. `settings -set` só muda a memória; `settings -apply <module>` empurra a mudança para o subsistema que já a consumiu no boot, sem reiniciar — ver "Aplicar configuração sem reboot" abaixo.
- **SDCard** / **USBMassStorage**: Acesso ao cartão SD e transferência de propriedade exclusiva do FAT entre o robô e um host USB.
- **StateMachine**: Máquina de estados do robô, com transições e *callbacks* configuráveis. Além do estado atual, guarda pedido de transição (`state -set`), trava (`state -lock`) e um anel com as últimas 8 transições.
- **JobScheduler**: Comandos de shell disparados por tempo (`every`/`once`) ou por entrada em estado (`at`). C++ puro, sem FreeRTOS/SD/TinyShell — o relógio chega por parâmetro e a execução por *callback*, o que deixa a lib inteira coberta por `test/test_job_scheduler` no `env:native`. Um *job* é agendamento, não garantia de entrega: ocorrência que não coube na fila é contada e descartada, e atraso maior que um intervalo inteiro é descartado em vez de virar rajada.
- **SystemMonitor**: Saúde do sistema (CPU, memória, temperatura, uptime, carga e *stack* por task). Sempre compilado; o relatório periódico é silenciado com `timers.sysmon_freq_ms = 0`, sem tirar os comandos sob demanda do ar.
- **StaticObjects**: Inicializa e centraliza instâncias globais dos principais objetos (robô, sensores, motores, logger, etc.).
- **TinyShell**: Interpretador de linha de comando embarcado. Organiza comandos em módulos, suporta autocompletar (*auto-completion*), converte dinamicamente os tipos de argumentos de *strings* para os tipos esperados, valida a execução e lida com erros de forma segura (*try-catch*).

### Comandos agendados e script de boot

O shell deixou de ser só um alvo de comandos. O módulo `job` agenda linhas do próprio shell:

```
job -every 5000, sensor -position          # a cada 5 s, para sempre
job -repeat 200, 10, kalman -state         # 10 vezes, a cada 200 ms
job -once 3000, junkebox -play_builtin boot
job -at WAIT, sensor -calibrate            # a cada ENTRADA em WAIT
job -list / job -cancel <id> / job -cancel_all / job -stats
```

O comando agendado viaja como um único argumento e pode conter espaços; vírgulas dentro dele são escapadas com `\,` (o TinyShell separa argumentos em vírgulas não escapadas):

```
job -every 1000, robot -set_pwm_pair 40\, 40\, 500
```

Um arquivo de comandos no cartão roda com `job -run_file <path>`, e o arquivo `autoexec.job` na raiz roda sozinho no boot — é o que permite configurar o robô sem ninguém do outro lado. Linhas em branco e começadas por `#` são ignoradas; uma linha por passagem do `routine()`, o que impede um script longo de estourar a fila de 10 comandos.

Duas recusas deliberadas: **um job não agenda jobs** e **um script não roda comandos `job`** — as duas coisas se auto-replicam sem limite e encheriam as 8 vagas em poucas passagens.

### Aplicar configuração sem reboot

Antes, `settings -set` só mudava a cópia em memória — quase nada era relido depois do boot, então uma edição remota parecia ter funcionado e não tinha efeito nenhum. `settings -apply <module>` é a metade que faltava: o pedaço de código que sabe empurrar os valores de um módulo para o subsistema que já os consumiu na inicialização.

```
settings -set ekf_noise gyro_noise 0.01
settings -apply ekf_noise      # reconstrói o filtro agora, sem reboot
settings -apply_all            # roda todo aplicador registrado
settings -diff                 # o que difere entre memória e settings.conf
```

`RobotSettings` continua sem depender de nenhum outro módulo (ver a regra de acoplamento acima): quem sabe aplicar um módulo se registra nela via `register_applier`, ela não conhece EKF, OTA ou Logger por si.

Nem todo módulo pode ser aplicado em runtime, e `settings -apply` diz honestamente qual é o caso:

| Módulo | O que acontece em `settings -apply` |
|---|---|
| `timers` | Reinicia o timer do EKF com o novo `sample_micros` (`esp_timer_restart`, sem perder a periodicidade) e aplica `timezone` via `setenv`/`tzset` na hora, sem esperar o próximo `logger -set_datetime`. |
| `logger` | `set_flush_limits(max_chunks_per_flush, block_size)`. |
| `ota` | `OTAUpdater::configure()` de novo — muda os tempos, o *hostname* mDNS e a senha. **Não** move o canal ESP-NOW que já está no ar: `espnow_channel` aqui só muda para onde o `ota -cancel` restaura o rádio depois. |
| `ekf_noise` | Reconstrói o filtro inteiro (`Q`/`R` são `const` para a vida do `TinyEKF`), com a task do EKF suspensa durante a janela — é a única reconstrução deste tipo no firmware, documentada como tal. |
| `kinematics`, `error` | Não fazem nada porque já não precisam: `encoder_ppr`/`wheel_radius`/`error_blink_ms` são lidos direto de `settings.data()` a cada uso. Registrados como *no-op* só para a resposta ser "applied" em vez de "requires a reboot". |
| `sensor`, `pins_*` (7 grupos) | **Sem aplicador**: `settings -apply` responde `requires a reboot`. `ArraySensor` reconfigura ADC/GPIO no construtor e os grupos de pino dimensionam periféricos já energizados — não há caminho de reconfiguração viva para nenhum dos dois. |

`settings -revision` conta quantas vezes `save()` gravou o arquivo com sucesso **nesta sessão de boot** (zera a cada reboot, `load()` não mexe nele). É a forma barata de um cliente remoto perceber "a configuração mudou desde a última vez que eu olhei", sem reler as ~55 chaves de `settings -list_all` para comparar. Deliberadamente **não** foi enxertado em `ManifestResponder::kConfigRevision`: aquele campo do protocolo (BTP/docs/commands.md seção 3) documenta "o catálogo de tópicos/schemas mudou", que neste firmware é genuinamente fixo em tempo de compilação — reaproveitá-lo para outra coisa seria mudar unilateralmente a semântica de um campo do protocolo, a mesma classe de problema que `include/bally_channels.h` existe para evitar entre os três repositórios.

### Descoberta e diagnóstico

`sys -manifest` lista todo módulo e comando que o shell deste firmware conhece, para um cliente novo descobrir o catálogo em vez de embarcar uma lista copiada à mão que desatualiza. Construído em cima de `TinyShell::complete_line()` — a única introspecção **pública** que a biblioteca vendorizada expõe (`get_help()` e `get_expected_types()` são privados). Por isso o manifesto lista só nomes de módulo/comando, sem descrição nem tipo de argumento; combine com `help -complete` durante a digitação para o resto.

`sys -time_sync` sincroniza o relógio por SNTP (`pool.ntp.org`) **enquanto o OTA estiver conectado a uma rede Wi-Fi** — é a única rota deste firmware até um servidor NTP, já que o ESP-NOW não tem caminho até a internet. O horário sincronizado **não sobrevive a um reboot**: esta placa não tem RTC com bateria (ver o pinout em `include/Settings.h`), então cada boot começa do zero e precisa da sua própria sincronização.

Um `esp_register_shutdown_handler()` grava o *ring* de PSRAM retido no cartão antes de todo reboot **ordenado** (`sys -reboot`, `factory_reset`, o restart pós-upload do OTA) — mesmo raciocínio que já existia só para a transição TELEMETRY: o que ainda não saiu pelo rádio (possivelmente sem alcance) seria perdido quando o *ring* é limpo. **Isso não cobre uma queda (panic/abort)** — `esp_restart()` pula os *shutdown handlers* nesse caso, e cobrir essa metade exigiria uma partição de coredump, que este firmware ainda não tem (ver "O que ficou de fora" abaixo).

### Canal B (chave E) agora existe de ponta a ponta

O gap original ("canal B decifra mas a resposta sempre sela com a chave L, então quem mandou pelo canal B nunca consegue abrir a própria resposta") foi fechado, não só contornado:

- `RadioSeal::seal_e()`/`open_e()` espelham `seal()`/`open()`, lendo `key_e()` em vez de `key_l()`.
- `ROBOT::handleReceiveStatic` classifica todo `COMMAND` recebido com `bally::channel_of_peer(Vantage::Robot, header.source_id, dongle_source_id_)` **antes** de decifrar — `source_id` viaja em claro mesmo dentro de um frame selado — e escolhe `open()` ou `open_e()` de acordo. `dongle_source_id_` é derivado uma vez de `MAC_ADDR` (o mesmo build flag que já identifica a dongle) com a mesma conversão MAC→source_id que o robô já usa para a própria identidade — não é um `RobotSettings` novo.
- `CommandProcessor` agora sabe de qual canal cada requisição veio (`ResultView::channel`, guardado por pedido no cache de deduplicação) e `send_result()` sela a resposta com a chave **daquele** canal. Se a chave do canal específico não estiver configurada mas a do outro canal estiver, a resposta é **descartada**, nunca sai em claro nem selada com a chave errada — testado em `test/btp_integration` (`test_channel_b_reply_is_sealed_with_endpoint_key_not_link_key`, `test_channel_b_reply_without_endpoint_key_configured_is_dropped`).
- Deliberadamente **não** estendido a MANIFEST_REQUEST/SUBSCRIBE/UNSUBSCRIBE: essas respostas ainda saem sempre com a chave L. Alargar o canal B até elas exige o mesmo tratamento em `ManifestResponder`/`SubscriptionResponder`, não feito aqui.

`ota -scan` dispara uma varredura Wi-Fi de redes visíveis sem exigir nenhuma rede já cadastrada (ao contrário de `ota -start`, que precisa de pelo menos uma); `ota -scan_results` lista o que a última varredura viu (SSID, RSSI, aberta ou não). `job -save` grava todo job `every`/`at` ativo em `jobs.conf`, restaurado automaticamente no próximo boot (depois de `autoexec.job`); um job `once` nunca é salvo, porque seu atraso é relativo ao momento em que foi agendado — persisti-lo faria a mesma linha significar outra coisa depois de um reboot.

### O que ainda ficou de fora, e por quê

- **Política de autorização por canal (READ/CONFIG/ACT).** `bally_channels.h` documenta que a chave L (canal C, dongle) "só administra o link" — não é a chave "de administrador geral" que eu ia assumir por padrão. Isso inverte a suposição óbvia (canal C = confiável, canal B = restrito) o suficiente para que eu não adivinhasse a direção: perguntei, e a decisão foi **não implementar nenhuma política agora** e manter os dois canais com acesso igual, documentado explicitamente em vez de deixado implícito (ver `sec -channels`) — uma escolha deliberada de não fazer nada agora, não um esquecimento.
- **Coredump em flash + partição dedicada.** Mudar `partitions.csv` é a categoria de mudança mais arriscada que existe neste firmware — quem estiver rodando a tabela de partições antiga precisaria de um flash por cabo antes de aceitar qualquer OTA futuro. Sem acesso a hardware para validar isso de ponta a ponta, e sem pedido explícito para assumir esse risco às cegas, essa mudança continua fora.

Consequência prática: `sec -channels` reporta os dois canais como selados (`sealed=1`), mas avisa que uma requisição com **qualquer uma** das duas chaves executa qualquer comando, motores inclusive — essa lacuna de autorização **persiste**, por decisão, não por lacuna técnica.

### Outras Pastas
- **robot/**: Implementação dos estados (Setup, Wait, Calibrate, Debug, Run, Finish, Telemetry, Error).
- **include/**: Definições globais necessárias para configurar o robô.
- **utils/**: Contém as configurações globais das funcionalidades do robô, centralizadas no objeto `ROBOT`, que permite criar apenas uma instância. Esse objeto unifica o acesso aos sensores, motores, utilidades dos sensores, logger, controle e demais recursos.

### Organização e Acoplamento entre Módulos

Cada pasta em `lib/` é compilável e compreensível isoladamente. Duas regras mantêm isso:

1. **Dependência entre bibliotecas só quando justificada, e sempre explícita.** A maioria dos módulos (`Flags`, `StateMachine`, `HBridge`, `Encoder`, `ArraySensor`, `SystemMonitor`, `Format`) não inclui nenhuma outra biblioteca do projeto. Onde uma dependência é real — `Logger` grava no `SDCard`; `OTAUpdater` usa `SDCard`/`Flags_out`; `USBMassStorage` usa `SDCard` — o header só faz `class Nome;` (forward declaration) e o `.cpp` inclui o header completo. Isso limita o acoplamento à implementação, não à interface pública: quem só usa a classe por referência/ponteiro nunca precisa saber o que ela inclui por baixo.
2. **`RobotSettings` não depende de nenhum outro módulo do projeto.** É a camada mais "de baixo" da configuração em runtime (dados + persistência em `settings.conf`); quem lê valores dela (`OTAUpdater`, `ArraySensor`, `Logger`, ...) depende dela, nunca o contrário. Os valores padrão compartilhados com o `OTAUpdater` (tempos, canal do ESP-NOW, ...) vivem em `lib/OTAUpdater/OtaDefaults.h`, um header sem nenhuma dependência que os dois incluem.

**Comandos de shell: cada módulo registra os próprios.** Toda classe que expõe comandos de shell implementa `register_shell_commands(TinyShell&, ...)` no seu próprio `.cpp`, recebendo por parâmetro só o que precisa (ex.: `OTAUpdater::register_shell_commands` recebe `Logger&`, `SDCard&`, `USBMassStorage&` e um `std::function<bool()>` para consultar se algum teste de DEBUG está ativo — sem precisar saber que "teste de DEBUG" é um conceito do `ROBOT`). `ROBOT::startWrappers()` (`utils/BallyRobot/BallyRobotShell.cpp`) é só a lista dessas chamadas de composição.

Alguns módulos de shell moram no próprio `ROBOT` (`registerSystemCommands`/`registerRobotIOCommands`/`registerKalmanCommands`/`registerDebugCommands`, em `utils/BallyRobot/BallyRobotShell.cpp`), por não terem dono natural fora dele:
- **`robot`** (btn/ssr/set_pwm/set_led): aciona os `Flags_in`/`Flags_out`/`Flags_pwm` que o próprio `ROBOT` compõe; não há uma biblioteca "dona" além dele.
- **`kalman`** (estado do EKF + log periódico): o filtro (`TinyEKF`) é uma dependência externa vendorizada, sem wrapper próprio no `lib/`; quem possui a instância, o timer de amostragem e os vetores de controle/medição é o `ROBOT`.
- **`debug`** (`test_arr_sensor`/`test_encoder`): o agendamento e o *gate* (estado DEBUG + USB ocioso) são aplicados uniformemente sobre vários sensores a partir de estado privado do `ROBOT`, não pertencem a nenhum sensor individual.
- **`sys`** (identidade, saúde e ciclo de vida): cruza `esp_system`, `esp_ota_ops`, `BtpEndpoint`, `SystemMonitor` e `StateMachine` de uma vez; nenhuma lib isolada responde "quem sou eu e como estou".
- **`job`** (comandos agendados + script do SD): o `JobScheduler` é propositalmente livre de TinyShell para ser testável no host, e o executor de script precisa do cartão SD e da fila de comandos, que são do `ROBOT`.
- **`link`/`telemetry`/`sec`** (rádio, protocolo e chaves): todas as libs que eles leem — `TxScheduler`, `RxRouter`, `CommandProcessor`, `TelemetryPublisher`, `KeyStore` — são compiladas pelo `env:native`, onde o TinyShell não existe. Registrar do lado do `ROBOT` é o que mantém `pio test -e native` de pé.

Sobre `motion`: o `drive` recebe **comando normalizado (-100..100), não m/s**. Este firmware não tem modelo de motor calibrado — `EKF_K_R`/`EKF_K_L` são 1.0 de placeholder e `control_input[]` é PWM cru (ver `sampleEKF`), então converter metro por segundo em duty seria unidade inventada. A mistura diferencial é real; só a escala não é calibrada, e `motion -limits` diz isso no output.

`motion -disarm` é aplicado dentro do `setOutputs()`, não só onde o comando é aceito: enquanto desarmado a saída é zero a cada passagem, esteja o que estiver nas flags de PWM. É o que faz dele chave geral em vez de pedido educado. Nasce armado, para não mudar o fluxo de bancada.

`motion -coast` precisa de um *latch* porque o `setOutputs()` reaplica as flags a cada passagem e `HBridge::applyPWM(0)` é **freio ativo** — um `coast()` solto seria desfeito em menos de um milissegundo. Qualquer PWM comandado vence o coast pendente.

Sobre `link -reset_stats`: ele **não zera contador nenhum**. Os contadores do `CONTROL/STATUS` são normativamente monotônicos desde o boot (commands.md seção 5), e zerá-los faria qualquer consumidor que calcula delta ver um valor negativo. O comando marca um ponto zero e o `link -delta` mostra a diferença desde ele.

Além disso, `BallyRobotShell.cpp` é **o único lugar** onde podem ser registrados comandos que operam sobre as bibliotecas compiladas pelo `env:native` (`BtpTransport`, `CommandProcessor`, `Format`, `KeyStore`, `ManifestResponder`, `RxRouter`, `StatusReporter`, `SubscriptionResponder`, `TelemetryPublisher`, `TxScheduler`): nenhuma delas pode incluir `TinyShell.h`, que não existe no build de host. Ver `CONTRIBUTING.md` e o comentário de cabeçalho do próprio arquivo.

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
- **ESP-NOW/BTP v1:** Utilizado para comunicação sem fio. `BTP` (`v1.0.1-beta`) é integrado via `lib_deps` do PlatformIO, fixado numa tag; nenhum `struct` C/C++ é transmitido.
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
