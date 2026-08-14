# ensaio_em_voo

Ensaios de voo da Equipe eVTOL ITA: manobras mínimas que exercitam **uma**
capacidade do drone de cada vez, para rodar antes de uma missão de competição.

## Por que existe

Uma missão de competição falha por muitos motivos ao mesmo tempo, e no meio da
prova não dá para saber qual. Se o drone não chega na base, foi a visão, a
projeção, o PID de alinhamento, a translação ou o cabeceio? Cada ensaio daqui
isola uma dessas pernas e produz um número no log — quanto errou, quanto
derivou — que serve de linha de base antes de a missão voar.

São propositalmente burros: sem visão, sem detecção, sem regra de negócio. Só
os estados do [`stdstates`](https://github.com/Equipe-eVTOL-ITA/stdstates) e o
mínimo de estado próprio para sequenciar a manobra e medir o resultado.

## Os ensaios

| Pacote | O que exercita | Manobra |
|---|---|---|
| `teste_grid_quadrado` | **Translação** — o drone chega onde foi mandado? | Percorre um quadrado de lado `lado_quadrado`, `n_voltas` vezes. |
| `teste_yaw` | **Cabeceio** — depois de girar, "para a frente" é a direção nova? | Gira 90°, avança `distancia_avanco`, gira 180°, avança de novo. `n_voltas` vezes. |

Os dois têm a mesma forma: um estado por perna da manobra, mais um
`DecisionState` que conta as voltas. Ver o comentário no topo de cada
`decision_state.hpp` — o do `teste_yaw` explica por que ele fica **antes** da
primeira volta, e não depois (o `TakeoffState` reancora o referencial do mundo,
então uma origem gravada antes da decolagem não vale nada).

## Como rodar

```bash
# Da raiz do workspace (ex.: ~/evtol/dev)
source scripts/ros_env.sh
./src/ensaio_em_voo/scripts/build.sh teste_yaw

# Em três terminais:
./src/ensaio_em_voo/scripts/simulate.sh      # PX4 SITL + Gazebo
./src/ensaio_em_voo/scripts/agent.sh         # Micro-XRCE-DDS Agent
ros2 launch teste_yaw simulation.launch.py
```

Para voo real, troque o launch por `flight.launch.py`. Os dois perfis leem
YAMLs diferentes (`config/simulation.yaml` e `config/flight.yaml`) — **trocar de
simulação para voo é trocar de arquivo, nunca editar o `.cpp`.**

Cada voo grava um rosbag em `~/evtol/mission_logs/<pacote>_<timestamp>/`.
Depois de um ensaio que deu errado, é a única forma de saber o que o drone via.

## Parâmetros

Todos no YAML de cada pacote. Os que definem a manobra:

- `teste_grid_quadrado`: `n_voltas`, `lado_quadrado`
- `teste_yaw`: `n_voltas`, `distancia_avanco`, `yaw_tolerance`,
  `rotation_timeout`, `advance_timeout`

Os ângulos do `teste_yaw` (90° e 180°) **não** são parâmetros: eles são a
definição da manobra, e trocá-los muda que ensaio é aquele. Estão no construtor
do `RotateState`.

## Acrescentando um ensaio

Use o `new_mission.sh` do `evtol-dev`, ou copie um dos pacotes existentes. A
estrutura é a padrão de missão do workspace (ver `docs/ARCHITECTURE.md` do
[evtol-dev](https://github.com/Equipe-eVTOL-ITA/evtol-dev)): `src/`,
`include/<pacote>/states/`, `launch/`, `config/`.

Um ensaio bom mede **uma** coisa e imprime o erro no log. Se ele precisa de
visão ou de regra de negócio para funcionar, provavelmente já é uma missão, e o
lugar dela é no repositório da competição.
