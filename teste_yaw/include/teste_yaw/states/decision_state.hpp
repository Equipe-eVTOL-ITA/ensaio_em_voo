#ifndef TESTE_YAW__STATES__DECISION_STATE_HPP_
#define TESTE_YAW__STATES__DECISION_STATE_HPP_

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/state.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"

/**
 * @brief Decide se ha outra volta da manobra a executar, e mede a anterior.
 *
 * Mesmo papel do DecisionState do teste_grid_quadrado, mas mais simples: la era
 * preciso rearmar as marcas de `is_visited` da rota, porque o WaypointListState
 * guarda o progresso nelas. Aqui nao ha rota -- cada perna e um estado com o
 * proprio on_enter, que recalcula tudo ao ser reentrado -- entao so falta
 * contar e medir.
 *
 * ELE FICA NO TOPO DO LACO, E ISSO NAO E ARBITRARIO
 *
 * A medida que interessa e de quanto o drone se afastou de onde a volta
 * comecou: a manobra (gira 90, anda r, gira 180, anda r) devolve o drone a
 * origem em teoria, e o que sobra e o erro acumulado. Logo, alguem tem de
 * gravar a origem ANTES da primeira volta.
 *
 * Esse alguem nao pode ser o construtor da FSM. O TakeoffState roda com
 * `set_home = true` e chama `setHomePosition`, que REANCORA o referencial do
 * mundo na posicao de decolagem -- toda coordenada lida antes disso passa a se
 * referir a um referencial que nao existe mais. Uma origem gravada no
 * construtor produziria erros de metros, sem erro nenhum no log. E o mesmo
 * mecanismo que fez o drone da fase 1 pousar na mesma base indefinidamente.
 *
 * Com este estado no topo do laco, a primeira entrada acontece logo APOS a
 * decolagem, ja no referencial definitivo. Dai a assimetria da logica abaixo:
 * a primeira entrada so grava a origem, e as seguintes e que fecham uma volta.
 *
 * Contrato com a blackboard:
 *
 *   entrada  "n_voltas"  float
 *
 * Outcomes: "NEXT LAP"   (comeca mais uma volta)
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

        float n_voltas = 0.0f;
        if (!stdstates::require(blackboard, drone, "n_voltas", n_voltas)) return "ERROR";

        const int total = static_cast<int>(n_voltas);
        const Eigen::Vector3d pos = drone->getLocalPosition();

        if (primeira_entrada_) {
            origem_ = pos;
            primeira_entrada_ = false;
            drone->log("Origem da manobra: {" + std::to_string(origem_.x()) + ", " +
                       std::to_string(origem_.y()) + "}");
        } else {
            voltas_feitas_++;
            drone->log(
                "Volta " + std::to_string(voltas_feitas_) + " de " +
                std::to_string(total) + " concluida. Erro acumulado: " +
                std::to_string((pos - origem_).head<2>().norm()) + " m.");
        }

        if (voltas_feitas_ >= total) return "LAPS DONE";
        return "NEXT LAP";
    }

private:
    // Membros, e NAO zerados em on_enter: a FSM e dona do unique_ptr do estado,
    // entao eles sobrevivem as reentradas. Zerar o contador ao entrar faria o
    // drone voar para sempre.
    int voltas_feitas_ = 0;
    bool primeira_entrada_ = true;
    Eigen::Vector3d origem_{0.0, 0.0, 0.0};
};

#endif  // TESTE_YAW__STATES__DECISION_STATE_HPP_
