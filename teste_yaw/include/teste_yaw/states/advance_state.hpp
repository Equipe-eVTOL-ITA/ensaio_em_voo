#ifndef TESTE_YAW__STATES__ADVANCE_STATE_HPP_
#define TESTE_YAW__STATES__ADVANCE_STATE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/state.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

#include "stdstates/blackboard_params.hpp"

/**
 * @brief Avanca `distancia_avanco` metros NA DIRECAO EM QUE O DRONE APONTA.
 *
 * Este e o estado que da sentido ao ensaio. Girar e depois andar para uma
 * coordenada fixa nao testaria nada de yaw: o drone chegaria la de qualquer
 * jeito, apontando para qualquer lado. O que se quer saber e se, DEPOIS de
 * girar, "para a frente" e mesmo a direcao nova -- e isso so aparece se o alvo
 * for calculado a partir do cabeceio atual.
 *
 * Se a rotacao anterior errou o alvo, o erro vira desvio lateral aqui, onde da
 * para medi-lo em metros em vez de radianos. Um erro de 5 graus vira 9 cm de
 * desvio a 1 m, e 44 cm a 5 m: aumentar `distancia_avanco` e o jeito de
 * amplificar um erro de cabeceio pequeno demais para ver no log.
 *
 * O alvo e congelado no on_enter, a partir do cabeceio de entrada. Recalcula-lo
 * a cada tick faria a trajetoria curvar junto com qualquer oscilacao de yaw,
 * e a curva esconderia justamente o erro que se quer medir.
 *
 * Contrato com a blackboard:
 *
 *   entrada  "distancia_avanco"        float  m
 *            "position_tolerance"      float  m
 *            "max_horizontal_velocity" float  (passo por tick; ver abaixo)
 *            "advance_timeout"         float  s
 *
 * Outcomes: ""          (avancando)
 *           "ADVANCED"  (chegou)
 *           "ERROR"     (parametro ausente, ou estourou o timeout)
 */
class AdvanceState : public fsm::State {
public:
    AdvanceState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        ok_ = false;

        auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ptr == nullptr || *drone_ptr == nullptr) return;
        drone_ = *drone_ptr;

        drone_->log("");
        drone_->log("STATE: ADVANCE");

        if (!stdstates::require(blackboard, drone_, "distancia_avanco", distancia_)) return;
        if (!stdstates::require(blackboard, drone_, "position_tolerance", tolerancia_)) return;
        if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", passo_)) return;
        if (!stdstates::require(blackboard, drone_, "advance_timeout", timeout_)) return;

        partida_ = drone_->getLocalPosition();
        yaw_ = static_cast<float>(drone_->getOrientation()[2]);

        // NED: x aponta para o Norte, y para o Leste, e o yaw e medido de x
        // para y. Logo a frente do drone e (cos yaw, sin yaw, 0). O z do alvo e
        // o z de partida: esta perna e horizontal.
        alvo_ = partida_ + Eigen::Vector3d(
            distancia_ * std::cos(yaw_),
            distancia_ * std::sin(yaw_),
            0.0);

        drone_->log(
            "Avancando " + std::to_string(distancia_) + " m no rumo " +
            std::to_string(yaw_ * 180.0f / static_cast<float>(M_PI)) + " graus.");

        inicio_ = std::chrono::steady_clock::now();
        ok_ = true;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (!ok_) return "ERROR";

        // O helper do drone_lib ja faz o passo de comprimento limitado e ja
        // segura o yaw durante a translacao -- travar o cabeceio aqui importa,
        // senao o drone giraria enquanto anda e a perna sairia curva.
        if (move_local_constant_step(drone_, alvo_, passo_, tolerancia_, yaw_)) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);

            const double percorrido = (drone_->getLocalPosition() - partida_).head<2>().norm();
            drone_->log(
                "Avanco concluido. Percorridos " + std::to_string(percorrido) +
                " m dos " + std::to_string(distancia_) + " m pedidos.");
            return "ADVANCED";
        }

        const std::chrono::duration<float> decorrido =
            std::chrono::steady_clock::now() - inicio_;
        if (decorrido.count() > timeout_) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            const double faltando = (alvo_ - drone_->getLocalPosition()).head<2>().norm();
            drone_->log(
                "ERRO: o avanco nao terminou em " + std::to_string(timeout_) +
                "s. Faltavam " + std::to_string(faltando) + " m.");
            return "ERROR";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d partida_{0.0, 0.0, 0.0};
    Eigen::Vector3d alvo_{0.0, 0.0, 0.0};
    float yaw_ = 0.0f;
    float distancia_ = 0.0f;
    float tolerancia_ = 0.0f;
    float passo_ = 0.0f;
    float timeout_ = 0.0f;
    std::chrono::steady_clock::time_point inicio_;
    bool ok_ = false;
};

#endif  // TESTE_YAW__STATES__ADVANCE_STATE_HPP_
