# Vários links num único `btp::Node` (item 4 do BTP)

**Status:** proposta, antes de qualquer código.
**Branches:** `feature/btp-multi-link-node` (bally_OS) e `feature/node-multi-link` (BTP, a criar).

## 1. Problema

Hoje o robô tem **três** `btp::StaticNode` (`node_`, `tcp_node_`, `ble_node_`), três `NodeConfig`
(`RobotLink`, `RobotTcpLink`, `RobotBleLink`) e, em volta disso:

- `tcp_node_`/`ble_node_` são `std::optional` recriados a cada conexão: `emplace` → `populateCatalog` →
  `begin` → `serve_catalog` → `bind` de 4 `BtpEndpoint` → `bind_*_target` no `TelemetryPublisher` e no
  `TerminalResponder` → `enable_session`/`arm_session`. A desconexão desfaz tudo na ordem inversa.
- `tcp_session_generation_`/`ble_session_generation_` e os campos `from_tcp`/`tcp_generation`/`from_ble`/
  `ble_generation` do `QueuedCommand` existem só porque `runShell()` pode rodar depois que o Node da
  conexão foi destruído: sem eles haveria *use-after-free* ou resposta indo para a conexão errada.
- `onTcpReceive`/`onBleReceive` e `processTcpCommandRequest`/`processBleCommandRequest` são cópias quase
  literais uma da outra e do caminho ESP-NOW.
- O catálogo é populado três vezes, e o `std::optional` reserva RAM estática para os três Nodes inteiros
  mesmo que só um transporte direto seja usado por boot.

O motivo de serem três Nodes é um só: `Node::enable_session()` põe **o Node inteiro** atrás do handshake
HELLO. Se o ESP-NOW (que não tem sessão) e o TCP dividissem o Node, o tráfego do dongle ficaria bloqueado
esperando um HELLO que nunca chega.

## 2. Ideia

Separar o que é **do nó** (quem o robô é e o que ele serve) do que é **de cada link** (um caminho até um
peer). Um Node, N links. A sessão passa a ser uma propriedade do link, não do Node.

## 3. O que é compartilhado e o que é de cada link

| Estado | Onde fica | Por quê |
|---|---|---|
| Identidade (`source_id`, `boot_id`) | **Node** | É uma identidade só. |
| Contador de sequência (`btp::Endpoint`) | **Node** | Um espaço de sequência por identidade é o que o comentário de `BtpEndpoint` já exige. `reserve_sequence()` já é atômico. Verificado: nem TraceView nem o dongle tratam buraco de sequência como perda. |
| Catálogo servido, `source_info`, uuid/nome/role | **Node** | Idêntico nos três hoje. Só é lido depois do boot. |
| Registro de `on_publish` | **Node** | Sai do catálogo. |
| Scratch de MANIFEST_DATA + seal scratch | **Node**, protegido por um *lock hook* opcional | É a maior economia de RAM (2 × ~3,8 KB). Só é usado ao montar MANIFEST_DATA, que é raro. Ver §5. |
| `DedupCache` de comandos | **Node** | A spec diz que a deduplicação vale pelo boot do executor, não pela sessão (session-and-terminal §5.3). No robô ela já é compartilhada, via `CommandProcessor`. |
| Handlers de terminal/comando | **Node**, e o handler recebe o link de origem | Um único `TerminalResponder`; ele precisa saber por qual link responder. |
| `send()` e `TransportLimits` | **Link** | São o próprio transporte. |
| `seal`/`open`/`reply_seal`/`accept_cleartext` | **Link** | A política de chave muda por link. No ESP-NOW ela classifica o canal (B/E ou C/L), no TCP e no BLE é sempre B/E. É exatamente o que hoje separa `RobotLink` de `RobotTcpLink`. |
| `Receiver` (slots de remontagem) + `rx_buffer`/`open_buffer` | **Link** | Fragmentos de links diferentes não podem se misturar, e o reset de um link só descarta as remontagens pendentes dele. |
| Sessão responder (liga/desliga + deadline do HELLO) | **Link** | ESP-NOW desligado; TCP e BLE ligados. É o que resolve §1. |
| `SubscriptionTable` | **Link** | Uma concessão pertence a um peer num link. O fan-out da telemetria é por link, e a desconexão zera só a tabela daquele link. |
| `last_*_event`, `receive_outcome`, stats | **Link** | São escritos por quem chama `receive` naquele link. Ver §5. |
| **Época** (novo, atômico) | **Link** | É incrementada a cada `reset_link()`. Substitui os contadores de geração. |
| Initiator / `SubscriptionClient` / `CommandClient` | Só no link 0 nesta etapa | O robô não usa. Dongle e TraceView continuam single-link (ver §6). |

## 4. API proposta (BTP 2.47.0, minor, compatível)

```cpp
// O que hoje é NodeConfig, sem identidade e sem handlers de nível do nó:
class NodeLink {
public:
    TransportLimits transport{};
    virtual bool send(const std::uint8_t* frame, std::size_t n) = 0;
    virtual bool has_seal() const noexcept; virtual bool seal(...);
    virtual bool has_open() const noexcept; virtual bool open(...);
    virtual bool accept_cleartext(const Header&);
    virtual void reply_seal(const Header&, EndpointSealFn*, void**);
};

// NodeConfig continua existindo = NodeLink (link 0) + identidade + handlers + lock hook.
class NodeConfig : public NodeLink {
    std::uint32_t source_id, boot_id;
    /* has_terminal/terminal, has_command/command, has_clock/clock  (inalterados) */
    virtual void lock() {}   virtual void unlock() {}   // §5, no-op por padrão
};

struct LinkRef { std::uint8_t link; std::uint32_t epoch; };   // "este link, nesta conexão"

// Novos em Node (link 0 == o cfg do construtor; a API atual inteira continua falando do link 0):
bool   attach_link(std::uint8_t link, NodeLink& cfg);   // link 1..Links-1
NodeRx receive_on(std::uint8_t link, const std::uint8_t* d, std::size_t n,
                  std::uint64_t now_ms, ReceivedMessage* out);   // + overload DecodedFrame
void   enable_session_on(std::uint8_t link, const Hello&, std::uint64_t hello_deadline_ms);
void   arm_session_on(std::uint8_t link, std::uint64_t now_ms);
void   reset_link(std::uint8_t link);   // remontagem, sessão -> Idle, assinaturas; epoch++
LinkRef link_ref(std::uint8_t link) const;   // {link, época atual}
bool   link_current(const LinkRef&) const;   // época ainda bate?
SubscriptionTable* subscriptions(std::uint8_t link);
std::uint8_t current_link() const;   // válido dentro de terminal()/command()

// StaticNode ganha Links como último parâmetro de template (default 1 = hoje, byte a byte).
```

Todo o storage por link (slots do `Receiver`, `rx`/`open`, subscriptions) mora no `StaticNode`,
dimensionado por `Links`. Os slots têm o mesmo tamanho em todos os links, o que já é o caso hoje
(`ProtocolNode` é um alias só).

## 5. Concorrência

Hoje cada Node é tocado por uma task diferente: o callback do ESP-NOW, a task `btp_tcp` e o host do
NimBLE. **A regra continua a mesma: um link, um contexto.** `receive_on(L)` só mexe no estado do link L.
O que é compartilhado é só leitura depois do boot (identidade, catálogo), atômico (sequência, época) ou
usado dentro de `cfg.lock()/unlock()`, que é só o scratch de MANIFEST_DATA. O robô implementa o lock com
um mutex FreeRTOS, e a biblioteca continua sem RTOS. Nenhum caminho de telemetria ou de comando passa
pelo lock.

Janela conhecida: entre `link_current(ref)` e o `send` da resposta, o link pode ser resetado. O frame então
cai no `send()` de um transporte sem cliente (é recusado) ou, no pior caso, chega a um cliente recém-conectado
como um COMMAND_RESULT órfão. Essa janela **já existe hoje** com os contadores de geração, e este desenho
não a piora.

## 6. Efeito no bally_OS

Links fixos: `0 = ESP-NOW`, `1 = TCP`, `2 = BLE`.

- **Sai:** `tcp_node_`, `ble_node_`, os dois `*_session_generation_`, os 4 campos de origem/geração do
  `QueuedCommand` (viram um `btp::LinkRef reply_link`), `processTcp/BleCommandRequest`, `onTcp/BleReceive`
  (viram `onLinkReceive(link, ...)`), o populate/begin/bind por conexão e os pares `bind_*_target`/
  `unbind_*_target`.
- **`RobotTcpLink` + `RobotBleLink` → `RobotDirectLink`**, com duas instâncias. As classes só diferem no
  `send()`.
- **Os `BtpEndpoint` (`protocol_tcp_`, `protocol_ble_` e os de telemetria) passam a ser ligados uma vez só,
  no boot**, ao endpoint único. O endpoint não é mais destruído, então o risco de *use-after-free* deixa de
  existir por construção. O de resposta para a conexão errada fica com a época.
- **Conectar** vira `reset_link(L)` + `enable_session_on` + `arm_session_on`. **Desconectar** vira
  `reset_link(L)` + `terminal_responder.on_link_reset(L)`, que ainda precisa despejar os slots daquela origem.
- **`TelemetryPublisher`:** os alvos passam a ser indexados por link e ligados no boot. A tabela de
  assinaturas do link é zerada no reset, então a telemetria para sozinha, sem `unbind`.
- **Fora do escopo:** dongle e TraceView continuam no Node de um link (a API atual não muda). O `tick()` dos
  links com sessão também fica como está: hoje ninguém chama `tick()` no robô, e ligar o watchdog de
  sessão agora derrubaria sessões TCP/BLE ociosas. Isso fica para um item à parte.

**RAM (medido, `pio run -e esp32-s3`):** 227.584 → 210.512 bytes estáticos, ou seja, **−17.072 bytes**
(69,5% → 64,2%). A economia vem de 2 × catálogo, 2 × scratch de manifest, 2 × seal scratch e 2 × caches de
comando. A flash fica praticamente igual.

**Achado fora do escopo:** o caminho principal de TCP/BLE do robô entrega os bytes crus ao `receive()` e
envia frames crus, mas a spec (fragmentation-and-transports §8/§9) e o TraceView usam stream enquadrado com
COBS. Isso já existia antes deste item, e só o `TcpBusyResponder` faz COBS. Precisa ser confirmado em
hardware e corrigido à parte, de preferência com um enquadramento de stream por link no próprio BTP.

## 7. Conflito com trabalho em andamento

No working tree do `main` há mudanças não commitadas de outra pessoa em `TelemetryPublisher`
(`collect_targets`/agregação por destino), `TcpBtpServer` (buffers de RX fora da stack), no pin do BTP e em
`SetupCommTrigger`. Esta branch foi criada **a partir do `main` commitado, num worktree separado**
(`../bally_OS-multilink`), sem tocar nesses arquivos. `TelemetryPublisher` fica para o **último** passo, e
só depois que aquele trabalho for commitado, para fazer rebase em cima dele em vez de competir.

## 8. Passos (um commit cada)

1. **BTP:** extrair `NodeLink` e mover o estado por link para dentro de um `LinkState` (o link 0 é o `cfg`).
   Sem mudança de comportamento; a suíte atual passa inalterada.
2. **BTP:** `attach_link`/`receive_on`/sessão por link/`reset_link` + época/`LinkRef`/`current_link`, mais
   `StaticNode<..., Links>` e o lock hook. Testes novos: ESP-NOW sem sessão + TCP com sessão no mesmo Node;
   reset isola o link; época velha recusada; sequência única entre links.
3. **BTP:** `docs/library.md` + release 2.47.0.
4. **bally_OS:** subir o pin para 2.47.0; um Node com 3 links; `RobotDirectLink`; `LinkRef` no `QueuedCommand`;
   unificar os caminhos de receive e de comando.
5. **bally_OS:** `TerminalResponder` e `TelemetryPublisher` por link, depois do rebase (§7).
6. `pio test -e native` + `pio run -e esp32-s3`, e teste em hardware (ESP-NOW + TCP, ESP-NOW + BLE), que
   fica com você.
