# Como organizar o código deste projeto

Este documento é a política de organização do firmware do Bally Robot: como criar uma biblioteca nova, como gerenciar memória e *tasks* do FreeRTOS, e as convenções gerais do projeto. O objetivo é manter dependências mínimas, explícitas e direcionais, preservando `utils/BallyRobot` (a classe `ROBOT`) como dono e coordenador central do robô — ver [README.md](README.md#organização-e-acoplamento-entre-módulos).

Regra geral: deve ser possível identificar onde um comportamento vive, de quais dependências precisa, quem possui seu estado e como esse estado muda. Prefira a solução mais simples que deixe esses contratos explícitos, sem abstrações para casos hipotéticos.

## BallyRobot como dono e coordenador

**A existência de uma classe central é uma decisão do projeto.** O `ROBOT` cria e possui
os componentes, conecta callbacks, coordena modos de operação e define a ordem de
inicialização e recuperação. Pode conhecer todos os subsistemas. Seu tamanho, sozinho,
não justifica desmontar essa organização.

- Métodos de coordenação devem revelar o roteiro do robô: ler entradas, avaliar estado,
  aplicar saídas, solicitar publicação. Mantenha um nível coerente de detalhe por função.
- Cada subsistema mantém seu estado, suas invariantes e os detalhes de implementação.
  O chamador solicita uma operação, como encerrar uma sessão; o próprio subsistema
  garante a ordem de invalidar referências, limpar filas e liberar recursos.
- Coordenação entre componentes pode permanecer no `ROBOT`. Parsing de pacotes,
  manutenção de sessões e interpretação de scripts têm donos específicos quando
  constituem comportamentos próprios. Um componente extraído continua pertencendo ao
  `ROBOT` e recebe somente as dependências necessárias, nunca um `ROBOT&` para acessar tudo.
- Não crie uma classe para cada grupo de funções. Reutilize os módulos existentes;
  extraia um componente quando isso delimitar uma responsabilidade ou ciclo de vida real.
  Ele não precisa ser uma biblioteca genérica nem ter uma interface virtual.
- É permitido separar implementações em arquivos como `BallyRobotTasks.cpp` e
  `BallyRobotShell.cpp`. Essa divisão facilita navegação, mas não isola estado nem resolve
  acoplamento por si só. No header, agrupe API pública, inicialização, entradas de tarefas,
  componentes e estado de coordenação com seu contrato de sincronização.

## Limites entre componentes

- **Responsabilidade delimitada:** descreva o propósito de cada módulo em uma frase.
  Funcionalidades que mudam por motivos independentes não devem compartilhar estado
  apenas por conveniência. Nomes como `Manager`, `Utils` ou `Support` precisam de escopo claro.
- **Dependências explícitas:** não introduza ciclos de dependência entre módulos, inclusive
  nos `.cpp`. Callbacks devem expor operações específicas, com contexto de execução e
  possibilidade de reentrada definidos; não servem para esconder dependência do objeto inteiro.
- **Fronteiras de entrada:** shell, ESP-NOW, TCP e BLE adaptam entradas e saídas.
  Regras equivalentes de comandos, assinaturas e publicação têm uma implementação comum;
  particularidades do transporte ficam no adaptador. Comandos de shell validam argumentos,
  chamam operações e apresentam resultados, sem duplicar a lógica do subsistema.
- **Fonte de verdade:** use IDs, constantes e codecs canônicos do BTP. Não copie offsets
  ou layouts já implementados. Ao adicionar um destino de telemetria, envio, contagem,
  taxa e limpeza devem considerar o mesmo conjunto de destinos, evitando listas paralelas.
- **Extração com propósito:** compartilhe código quando ele representar a mesma regra,
  não apenas porque duas funções se parecem. Uma regra de protocolo duplicada deve ser
  consolidada; uma abstração genérica para transportes futuros não é necessária.
- **Fluxo explícito:** separe parsing, decisão e execução quando forem etapas distintas.
  Sessões e inicialização com várias fases precisam de estados e transições definidos,
  incluindo falha, timeout, fila cheia, desconexão e repetição. Evite combinações de flags
  que permitam estados contraditórios. Falhas parciais devem ter recuperação ou estado
  degradado explícito antes de continuar a operação.

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
   - Comandos de coordenação e adaptadores de módulos portáveis podem ficar em `utils/BallyRobot/BallyRobotShell.cpp`. Isso não transfere para o shell a implementação dos subsistemas: o wrapper chama operações específicas e apresenta seus resultados.
6. **Nomeie de forma consistente**: PascalCase para classe/arquivo/pasta, mesmo nome nos três.
7. **Não precisa registrar nada em `platformio.ini`** — o Library Dependency Finder do PlatformIO encontra `lib/*` sozinho a partir do primeiro `#include <NomeDaLib.h>`.
8. **Antes de considerar uma mudança de código pronta**, rode `pio run -e esp32-s3` **e** `pio test -e native`. Mudanças exclusivamente documentais exigem revisão do diff, links e coerência das regras, sem build de firmware. Evite colocar funcionalidade atrás de uma flag de build desligada por padrão: código assim nunca é compilado no binário que roda em campo (foi o caso do `SystemMonitor`, que ficou fora do firmware por um `;` no `platformio.ini`). Se algo precisa poder ser desligado, faça disso um *setting* em runtime com um valor sentinela documentado.

### Como decidir "isso é da lib ou do ROBOT?"

- Lógica que só depende do estado da própria lib → mora na lib.
- Lógica que decide **quando os componentes colaboram** → pode morar no `ROBOT`, usando suas APIs e callbacks específicos. Exemplo: ao entrar em RUN, cancelar testes e preparar os motores.
- Lógica que implementa **como um subsistema funciona** → mora nesse subsistema, mesmo que use várias libs. Exemplo: invalidar e encerrar uma sessão BLE ou interpretar um script. O número de dependências não torna o `ROBOT` seu dono automático.

## Gerência de memória

- **Alocação estática para tudo que vive o programa inteiro**: stacks de task, TCBs, buffers de módulos (`RobotSettings::data_`, o array cíclico do `Logger`, etc.). Siga os tamanhos já definidos em `include/Settings.h` (`M2KB`, `M4KB`, `M8KB`, `M16KB`, `M32KB`) em vez de números mágicos.
- **`std::optional<T>` + `.emplace()`** para objetos cuja construção depende de configuração carregada do SD em runtime (pinos vêm de `settings.load()`, que só roda depois que o cartão monta) — ver `array_sensor`, `encoder_left/right`, `motor_left/right`, `imu`, `EKF` em `utils/BallyRobot/BallyRobot.h`. Nunca `new`/`delete` para isso.
- **PSRAM só para buffers grandes e de vida longa**, alocados uma vez com `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` (ver `Logger`), com o tamanho e o motivo documentados ao lado da constante (ex: `LOGGER_PSRAM_CAPACITY_BYTES` em `include/Settings.h`).
- **Sem alocação dinâmica dentro de ISR ou de qualquer função `IRAM_ATTR`.** Interrupções só tocam memória já alocada (ex: `Flags_in::setFlag` mexe só em campos already-alocados do objeto).
- **Toda struct/classe acessada por mais de uma task precisa declarar como se protege**, e o comentário deve dizer qual task pode chamar o quê:
  - `std::atomic<T>` com `memory_order` explícito para estado simples lido/escrito por tasks diferentes sem precisar de uma seção crítica maior (ver `StateMachine::current_state`, `OTAUpdater`'s `phase_`/`flashing_`, `USBMassStorage`'s flags de sessão).
  - `SemaphoreHandle_t` (mutex) quando a operação precisa ser exclusiva por mais que uma escrita de palavra — ver `Logger::wait_for_mutex()/check_mutex()/free_mutex()` guardando o *ring buffer*.
  - Comunicação entre ISR e task usa mecanismo adequado ao contexto: APIs `FromISR`, seção crítica apropriada ou atomics cuja implementação seja comprovadamente adequada à ISR. Tamanho de palavra e `volatile`, sozinhos, não são contrato de sincronização.
- **Tempo de vida faz parte da proteção.** Documente quem cria, modifica e destrói cada recurso e até quando referências emprestadas são válidas. Um ponteiro atômico não mantém o objeto vivo; desconexão e destruição precisam coordenar leitores e operações em andamento. Zerar um ponteiro ou conferir uma geração não basta se o recurso puder ser destruído entre a checagem e o uso.
- **Não aceite corrida "tolerada" como justificativa.** Prefira um único responsável pelas mudanças de estado, recebendo pedidos por fila, quando isso simplificar o fluxo. Vários campos que formam um estado coerente precisam de um protocolo conjunto, não apenas atomics independentes.
- **Liberação por escopo:** use guardas RAII para mutexes adquiridos e recursos temporários. Erros e retornos antecipados devem liberar recursos automaticamente. Não mantenha locks durante I/O demorado ou callbacks externos sem um contrato explícito de bloqueio e ordem dos locks.

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
- **Onde registrar comandos que operam sobre uma lib compilada em `env:native`.** Oito bibliotecas (`BtpTransport`, `CommandProcessor`, `Format`, `KeyStore`, `ManifestCatalog`, `StatusReporter`, `TelemetryPublisher`, `TxScheduler`) são compiladas pelo `env:native` para os testes de host, onde o TinyShell não existe. **Nenhuma delas pode ganhar `#include <TinyShell.h>`** — isso quebra `pio test -e native` em todas as suítes de uma vez, e o erro aparece como falha de *link*, sem apontar a causa. Comandos que operam sobre essas libs vivem em `utils/BallyRobot/BallyRobotShell.cpp`, que só o build ESP-IDF compila. É o mesmo raciocínio que `RadioSeal.h` documenta para manter `btp::aead` fora do `BtpTransport`.
- **`StateMachine::current_state` pode ser lido diretamente por qualquer lib** que precise saber o estado atual (ex: bloquear `calibrate` durante `RUN`) — é a única exceção deliberada à regra de "não leia um singleton global de fora": `StateMachine` já expõe isso como API pública estática, então depender dele é uma dependência normal de biblioteca, não um acesso escondido ao `ROBOT`.

## Evolução e revisão

- Aplique estas regras incrementalmente: não amplie a dívida na área alterada e corrija
  os limites necessários à mudança. Não exija uma reescrita geral nem um limite arbitrário
  de linhas por arquivo. Tamanho é um sinal para revisão; responsabilidade é o critério.
- Em extrações, preserve comportamento e distribuição de tarefas. Mudanças de timing,
  prioridade ou protocolo devem ser explícitas e verificadas separadamente.
- Comentários descrevem contratos atuais, unidades, limites e decisões não óbvias.
  Histórico de migração pertence ao Git ou a documentos de decisões. Ao substituir um
  fluxo, revise chamadores, testes e documentação e remova os caminhos realmente obsoletos.
- Teste o comportamento afetado, inclusive conexões entre componentes: assinatura →
  geração → envio, conexão → uso → desconexão e erro → recuperação. Novos transportes
  devem passar pelos mesmos cenários compartilhados, além de seus casos específicos.
- Filas e buffers têm capacidade e comportamento em saturação definidos. Ao ampliar
  payloads, considere o custo multiplicado por filas e sessões, a stack e a margem de
  heap necessária à operação. Não reserve memória para casos hipotéticos sem necessidade.
- Builds e testes nativos não comprovam timing, consumo máximo de memória ou operação
  de hardware. Quando afetados, registre a validação em placa ou o que ficou pendente.

Checklist antes de concluir uma alteração:

- Está claro onde o comportamento pertence e quais dependências utiliza?
- A coordenação no `ROBOT` está legível, com detalhes preservados nos subsistemas?
- Foi criada dependência escondida, ciclo ou regra duplicada?
- Quem possui o estado e garante sua validade entre tarefas?
- O fluxo de falha, saturação e desconexão está definido?
- O teste verifica o comportamento necessário, incluindo a integração alterada?
- Comentários e documentação correspondem ao resultado final?

## Onde isso vive

- Este arquivo (`CONTRIBUTING.md`) é a referência completa — comece por aqui ao criar algo novo ou em dúvida sobre onde algo deveria morar.
- [`README.md`](README.md#organização-e-acoplamento-entre-módulos) explica o raciocínio arquitetural (por que o projeto ficou assim) — este documento é mais "o que fazer", o README é mais "por que fazer assim".
