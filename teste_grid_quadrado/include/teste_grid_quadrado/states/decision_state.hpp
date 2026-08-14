#ifndef TESTE_GRID_QUADRADO__STATES__DECISION_STATE_HPP_
#define TESTE_GRID_QUADRADO__STATES__DECISION_STATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "fsm/state.hpp"
#include "drone/Drone.hpp"

#include "stdstates/arena_point.hpp"
#include "stdstates/blackboard_params.hpp"

/**
 * @brief Decide se ha outra volta a dar e, se houver, rearma a grade.
 *
 * POR QUE ISTO EXISTE
 *
 * O WaypointListState guarda o progresso da rota em `is_visited`, gravado
 * direto no vetor da blackboard -- ele le a lista por PONTEIRO, de proposito.
 * E ele nunca zera essas marcas: e isso que permite sair do estado e voltar
 * retomando de onde parou.
 *
 * A consequencia e que, sem alguem rearmar as marcas, a segunda entrada no
 * estado termina no primeiro tick. Todos os pontos ja estao visitados, o
 * `nextUnvisited()` devolve nullptr, e a FSM sai com "WAYPOINTS ENDED" sem o
 * drone ter se movido. O sintoma e uma volta so, sem erro nenhum no log.
 *
 * Este estado e esse alguem. Ele mora aqui, e nao no stdstates, porque
 * "quantas voltas" e decisao da missao, nao regra de navegacao. E por isso que
 * o stdstates nao precisa ser tocado para repetir a rota.
 *
 * Contrato com a blackboard:
 *
 *   entrada  "waypoints"  std::vector<ArenaPoint>  (REESCRITO: marcas zeradas)
 *            "n_voltas"   float
 *
 * Outcomes: "NEXT LAP"   (ainda falta volta; grade rearmada)
 *           "LAPS DONE"  (n_voltas completas)
 *           "ERROR"      (parametro ausente na blackboard)
 */
class DecisionState : public fsm::State {
public:
    DecisionState() : fsm::State() {}

    // Tudo em act(): este estado vive um tick so -- decide e transita na mesma
    // chamada -- entao separar on_enter/act nao compraria nada.
    std::string act(fsm::Blackboard &blackboard) override {
        auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ptr == nullptr || *drone_ptr == nullptr) return "ERROR";
        auto drone = *drone_ptr;

        drone->log("");
        drone->log("STATE: DECISION");

        // Por ponteiro: e o mesmo vetor que o WaypointListState le, e e ele que
        // precisa ser rearmado. Uma copia nao serviria para nada.
        auto *waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        if (waypoints == nullptr) {
            drone->log("ERRO: parametro ausente na blackboard: 'waypoints'");
            return "ERROR";
        }

        float n_voltas = 0.0f;
        if (!stdstates::require(blackboard, drone, "n_voltas", n_voltas)) return "ERROR";

        const int total = static_cast<int>(n_voltas);
        voltas_feitas_++;

        drone->log("Volta " + std::to_string(voltas_feitas_) + " de " +
                   std::to_string(total) + " concluida.");

        if (voltas_feitas_ >= total) return "LAPS DONE";

        for (auto &ponto : *waypoints) {
            ponto.is_visited = false;
        }
        return "NEXT LAP";
    }

private:
    // Membro, e NAO zerado em on_enter: a FSM e dona do unique_ptr do estado,
    // entao o contador sobrevive as reentradas. Zera-lo ao entrar faria o
    // drone voar para sempre.
    int voltas_feitas_ = 0;
};

#endif  // TESTE_GRID_QUADRADO__STATES__DECISION_STATE_HPP_
