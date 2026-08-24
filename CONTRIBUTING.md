# Como organizar o código deste projeto

Este documento é a política de organização do firmware do Bally Robot: como criar uma biblioteca nova, como gerenciar memória e *tasks* do FreeRTOS, e as convenções gerais do projeto. O objetivo é manter `lib/*` desacoplado e `utils/BallyRobot` (o *composition root*, a classe `ROBOT`) enxuto — ver [README.md](README.md#organização-e-acoplamento-entre-módulos) para o raciocínio por trás da estrutura atual.

Regra geral: se você está em dúvida sobre onde algo deveria morar, prefira a opção mais desacoplada que ainda resolve o problema sem inventar abstração para um caso hipotético.

## Nova biblioteca

Checklist para criar `lib/NomeDaLib/`:

1. **Uma pasta por biblioteca**, com `NomeDaLib.h` + `NomeDaLib.cpp`. Só pule o `.cpp` se a biblioteca for genuinamente header-only e sem estado (ex: [`lib/Format/Format.h`](lib/Format/Format.h)).
2. **No header, inclua só o que a interface pública exige.** Antes de escrever `#include <OutraLib.h>` num `.h`, pergunte: "preciso do tipo completo aqui, ou só de um ponteiro/referência?"
   - Só ponteiro/referência → `class OutraLib;` (forward declaration) no `.h`, `#include <OutraLib.h>` completo só no `.cpp`.
   - Precisa do tipo completo no header (ex: campo por valor, `constexpr`, `default member initializer`) → inclua, mas prefira incluir o menor header que define esse tipo, não a interface inteira de outra lib. Foi exatamente esse problema que motivou este documento: `RobotSettings.h` incluía `OTAUpdater.h` inteiro (que arrasta `esp_event.h`/`esp_http_server.h`) só para copiar 6 valores padrão — a correção foi extrair esses valores para [`lib/OTAUpdater/OtaDefaults.h`](lib/OTAUpdater/OtaDefaults.h), um header sem nenhuma dependência, que as duas libs incluem.
3. **Nunca inclua `BallyRobot.h`** a partir de uma lib em `lib/`. Se `lib/X` inclui `utils/BallyRobot/BallyRobot.h`, algo está invertido — é o `ROBOT` que deve conhecer `X`, nunca o contrário.
4. **Nunca inclua `include/Settings.h` "porque é mais fácil".** Se precisar de uma constante de lá (ex: um `#define` de tamanho de buffer), inclua só isso e deixe um comentário dizendo qual constante e por quê — não assuma que ela chega de graça por include transitivo de outra lib (foi outro bug real: `RobotSettings.h` usava `OTA_MDNS_NAME_MAX_LEN` sem nunca incluir `Settings.h` diretamente).
5. **Se a lib expõe comandos de shell**, implemente:
   ```cpp
   void NomeDaLib::register_shell_commands(TinyShell& shell, Logger& logger, /* + só o que essa lib realmente precisa */);
   ```
   no `.cpp`, chamado uma vez a partir de `ROBOT::startWrappers()`. Regras:
   - Receba `Logger&` (e qualquer outra lib de que dependa de verdade) por parâmetro — nunca leia um singleton global "por fora" (exceção: `StateMachine::current_state`, ver seção de convenções gerais).
   - Se o comando depende de uma regra que combina *outra* lib que essa não deveria conhecer (ex: "só posso agir se nenhum teste de DEBUG estiver ativo em outro sensor"), receba isso como `std::function<bool()>`/`std::function<void()>` injetado pelo `ROBOT` — não inclua a lib dona daquele estado só para checar uma condição.
   - Só deixe um módulo de shell inteiro em `ROBOT::registerRobotIOCommands/registerKalmanCommands/registerDebugCommands` (`utils/BallyRobot/BallyRobot.cpp`) quando ele não tiver dono natural — ver os três exemplos documentados lá (raw I/O dos `Flags`, EKF vendorizado sem wrapper próprio, *gate* de DEBUG compartilhado entre sensores).
6. **Nomeie de forma consistente**: PascalCase para classe/arquivo/pasta, mesmo nome nos três.
7. **Não precisa registrar nada em `platformio.ini`** — o Library Dependency Finder do PlatformIO encontra `lib/*` sozinho a partir do primeiro `#include <NomeDaLib.h>`.
8. **Antes de considerar pronto**, rode `pio run -e esp32-s3` **e** `pio test -e native`. Evite colocar funcionalidade atrás de uma flag de build desligada por padrão: código assim nunca é compilado no binário que roda em campo (foi o caso do `SystemMonitor`, que ficou fora do firmware por um `;` no `platformio.ini`). Se algo precisa poder ser desligado, faça disso um *setting* em runtime com um valor sentinela documentado (ex: `timers.sysmon_freq_ms = 0` silencia o relatório periódico sem tirar os comandos do ar).

### Como decidir "isso é da lib ou do ROBOT?"

- Lógica que só depende do estado da própria lib → mora na lib.
- Lógica que combina regras de duas ou mais libs sem um dono natural comum → mora no `ROBOT` (composition root), injetada nas libs via parâmetro/callback. É esperado que o composition root conheça todo mundo — é para isso que ele existe — mas ele não deve conter a lógica de negócio de nenhuma lib individual.

## Gerência de memória

- **Alocação estática para tudo que vive o programa inteiro**: stacks de task, TCBs, buffers de módulos (`RobotSettings::data_`, o array cíclico do `Logger`, etc.). Siga os tamanhos já definidos em `include/Settings.h` (`M2KB`, `M4KB`, `M8KB`, `M16KB`, `M32KB`) em vez de números mágicos.
- **`std::optional<T>` + `.emplace()`** para objetos cuja construção depende de configuração carregada do SD em runtime (pinos vêm de `settings.load()`, que só roda depois que o cartão monta) — ver `array_sensor`, `encoder_left/right`, `motor_left/right`, `imu`, `EKF` em `utils/BallyRobot/BallyRobot.h`. Nunca `new`/`delete` para isso.
- **PSRAM só para buffers grandes e de vida longa**, alocados uma vez com `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` (ver `Logger`), com o tamanho e o motivo documentados ao lado da constante (ex: `LOGGER_PSRAM_CAPACITY_BYTES` em `include/Settings.h`).
- **Sem alocação dinâmica dentro de ISR ou de qualquer função `IRAM_ATTR`.** Interrupções só tocam memória já alocada (ex: `Flags_in::setFlag` mexe só em campos already-alocados do objeto).
- **Toda struct/classe acessada por mais de uma task precisa declarar como se protege**, e o comentário deve dizer qual task pode chamar o quê:
  - `std::atomic<T>` com `memory_order` explícito para estado simples lido/escrito por tasks diferentes sem precisar de uma seção crítica maior (ver `StateMachine::current_state`, `OTAUpdater`'s `phase_`/`flashing_`, `USBMassStorage`'s flags de sessão).
  - `SemaphoreHandle_t` (mutex) quando a operação precisa ser exclusiva por mais que uma escrita de palavra — ver `Logger::wait_for_mutex()/check_mutex()/free_mutex()` guardando o *ring buffer*.
  - Campos simples tocados por uma única ISR e lidos por uma única task consumidora (ex: `Flags`) podem dispensar mutex/atomic explícito **só quando** o tamanho da escrita for atômico na arquitetura (byte/bool) — não copie esse padrão para dados maiores que uma palavra sem pensar de novo.

## Gerência de tasks (FreeRTOS)

- **Toda task nova usa `xTaskCreateStaticPinnedToCore`**, com stack e `StaticTask_t` estáticos declarados em `src/main.cpp` (nunca `xTaskCreate` dinâmico) — siga o padrão de `xRoutineStack`/`xRoutineBuffer` já existente.
- **Escolha o núcleo de acordo com a divisão já estabelecida** (ver README, "Fluxo Detalhado do Sistema"): núcleo 1 (`APP_CPU_NUM`) é só para a state machine, prioridade máxima, sem concorrência; núcleo 0 (`PRO_CPU_NUM`) é para tudo periférico/paralelo (shell, EKF, interrupções, rotina). Uma task nova quase sempre entra no núcleo 0, a menos que precise da mesma prioridade da state machine.
- **Todo loop infinito de task tem um `vTaskDelay`** (no mínimo `WDOG_TIMEOUT_TK`) — nunca um `while(true)` sem *yield*, para não travar o watchdog nem morrer de fome as outras tasks do mesmo núcleo.
- **Documente a prioridade escolhida** — ver o comentário de `start_freertos_tasks()` em `src/main.cpp` explicando por que `state_machine` (10) > `EKF` (4) > `routine` (3) > `shell` (2) > `interrupts` (0).
- **Antes de reduzir uma stack**, meça o *high water mark* de verdade (via `sysmon tasks`, sempre disponível) em vez de chutar.

## Convenções gerais

- **Configuração que pode mudar em campo é um campo de `SettingsData`** (`lib/RobotSettings`), nunca uma macro nova em `include/Settings.h`. `Settings.h` é reservado para o que é genuinamente compile-time — cada seção do arquivo já explica por que aquele valor específico não virou setting; siga o mesmo padrão de comentário se adicionar algo lá.
- **Toda mensagem relevante ao usuário do shell passa pelo `Logger`** (`insert_log`/`insert_logf`, ou `send_log_direct` quando a resposta não deve ficar retida no PSRAM — ver `ROBOT::sendNextShellOutputDirect`), com o `logType` certo (INFO/WARN/ERRO/DEBG/CMDO). `ESP_LOGx`/`printf` direto só antes do logger estar pronto (o *retry loop* inicial do `app_main`).
- **Todo comando de shell segue o padrão `RESULT_OK`/`RESULT_ERROR`**, com mensagem de erro específica dizendo a causa (não só "falhou") e log de sucesso via `insert_logf` quando fizer sentido para telemetria.
- **A saída de um comando é texto de log, não o resultado BTP.** O `COMMAND_RESULT` carrega no máximo 128 octetos e só transporta status e erro; o texto que o usuário lê sai em frames `LOG` separados, fragmentados em blocos de 210 octetos, disputando a fila de TX com a telemetria. Consequência prática: **nenhum comando pode despejar saída ilimitada**. Todo comando que lista algo precisa de um limite explícito de itens ou de um argumento de paginação/ritmo — veja `logger print_log` (`file_index,delay_msg_ms`) como o padrão a copiar.
- **Formato de saída: uma linha de pares `chave=valor` separados por espaço.** É o que permite ao TraceView e ao dongle parsearem a resposta sem heurística por comando. Listas usam uma linha por item, prefixada pelo índice (`0 name=... size=...`). Prefira nomes de chave iguais aos do `SettingsData`/da API quando existirem, em vez de inventar sinônimos.
- **Onde registrar comandos que operam sobre uma lib compilada em `env:native`.** Dez bibliotecas (`BtpTransport`, `CommandProcessor`, `Format`, `KeyStore`, `ManifestResponder`, `RxRouter`, `StatusReporter`, `SubscriptionResponder`, `TelemetryPublisher`, `TxScheduler`) são compiladas pelo `env:native` para os testes de host, onde o TinyShell não existe. **Nenhuma delas pode ganhar `#include <TinyShell.h>`** — isso quebra `pio test -e native` em todas as suítes de uma vez, e o erro aparece como falha de *link*, sem apontar a causa. Comandos que operam sobre essas libs vivem em `utils/BallyRobot/BallyRobotShell.cpp`, que só o build ESP-IDF compila. É o mesmo raciocínio que `RadioSeal.h` documenta para manter `btp::aead` fora do `BtpTransport`.
- **`StateMachine::current_state` pode ser lido diretamente por qualquer lib** que precise saber o estado atual (ex: bloquear `calibrate` durante `RUN`) — é a única exceção deliberada à regra de "não leia um singleton global de fora": `StateMachine` já expõe isso como API pública estática, então depender dele é uma dependência normal de biblioteca, não um acesso escondido ao `ROBOT`.

## Onde isso vive

- Este arquivo (`CONTRIBUTING.md`) é a referência completa — comece por aqui ao criar algo novo ou em dúvida sobre onde algo deveria morar.
- [`README.md`](README.md#organização-e-acoplamento-entre-módulos) explica o raciocínio arquitetural (por que o projeto ficou assim) — este documento é mais "o que fazer", o README é mais "por que fazer assim".
