#ifndef TESTE_YAW__STATES__ROTATE_STATE_HPP_
#define TESTE_YAW__STATES__ROTATE_STATE_HPP_

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
 * @brief Gira `graus` em torno do proprio eixo, sem sair do lugar.
 *
 * POR QUE COMANDAR YAW ABSOLUTO, E NAO VELOCIDADE ANGULAR
 *
 * Daria para girar com `setLocalVelocity(0, 0, 0, taxa)`, como faz o
 * YawSweepState. Mas este pacote e um ENSAIO: ele existe para responder "o
 * drone faz o que as missoes vao mandar?" antes de a missao voar. E as missoes
 * nao comandam taxa -- o WaypointListState, o GoToState e o
 * move_local_constant_step todos mandam yaw ABSOLUTO junto com a posicao, via
 * `setLocalPosition(x, y, z, yaw)`. Entao e esse o caminho que este ensaio
 * exercita. Testar a taxa mediria uma coisa que nenhuma missao usa.
 *
 * POR QUE A POSICAO E CONGELADA
 *
 * O setpoint de posicao repetido a cada tick e a posicao de ENTRADA no estado,
 * nao a posicao atual. Realimentar a posicao atual deixaria a deriva livre para
 * se acumular: cada tick aceitaria como bom o lugar para onde o drone tivesse
 * escorregado. Com o ponto congelado, o PX4 tem de voltar -- e a deriva medida
 * no fim e a deriva de verdade, que e metade do que este ensaio quer saber.
 *
 * SOBRE OS 180 GRAUS
 *
 * O alvo e absoluto, entao quem escolhe o sentido da rotacao e o PX4, pelo
 * caminho mais curto. Em exatamente 180 graus os dois caminhos empatam e o
 * sentido pode variar entre execucoes. Isso nao e defeito: e o que uma missao
 * veria ao mandar meia volta. Se o log mostrar o drone hesitando no comeco da
 * meia volta, e este empate.
 *
 * Contrato com a blackboard:
 *
 *   entrada  "yaw_tolerance"     float  rad
 *            "rotation_timeout"  float  s
 *
 * Outcomes: ""         (girando)
 *           "ROTATED"  (chegou ao alvo dentro da tolerancia)
 *           "ERROR"    (parametro ausente, ou estourou o timeout)
 */
class RotateState : public fsm::State {
public:
    /// O angulo vem do CONSTRUTOR, e nao do YAML, de proposito: 90 e 180 sao a
    /// definicao da manobra que este ensaio executa, nao um ajuste. Trocar o
    /// numero muda que teste e este. Ja `n_voltas` e `distancia_avanco` sao
    /// ajustes de verdade, e esses estao no YAML.
    explicit RotateState(float graus)
        : fsm::State(), delta_(graus * static_cast<float>(M_PI) / 180.0f), graus_(graus) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        ok_ = false;

        auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ptr == nullptr || *drone_ptr == nullptr) return;
        drone_ = *drone_ptr;

        drone_->log("");
        drone_->log("STATE: ROTATE " + std::to_string(static_cast<int>(graus_)));

        if (!stdstates::require(blackboard, drone_, "yaw_tolerance", tolerancia_)) return;
        if (!stdstates::require(blackboard, drone_, "rotation_timeout", timeout_)) return;

        pos_travada_ = drone_->getLocalPosition();
        const float yaw_inicial = static_cast<float>(drone_->getOrientation()[2]);

        // Normalizar o alvo e obrigatorio: yaw vive em (-pi, pi], entao
        // 2.6 rad + pi/2 daria 4.17, um valor que a leitura NUNCA assume.
        // Sem isto o erro nunca fecha e o estado gira para sempre. E o mesmo
        // bug que o YawSweepState documenta.
        alvo_ = normalizeYawError(yaw_inicial + delta_);

        inicio_ = std::chrono::steady_clock::now();
        ok_ = true;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (!ok_) return "ERROR";

        const float yaw  = static_cast<float>(drone_->getOrientation()[2]);
        const float erro = normalizeYawError(alvo_ - yaw);

        if (std::abs(erro) < tolerancia_) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);

            // A medida que justifica o ensaio: quanto sobrou de erro de
            // cabeceio, e quanto o drone escorregou enquanto girava.
            const double deriva = (drone_->getLocalPosition() - pos_travada_).head<2>().norm();
            drone_->log(
                "Rotacao de " + std::to_string(static_cast<int>(graus_)) +
                " graus concluida. Erro final: " +
                std::to_string(erro * 180.0f / static_cast<float>(M_PI)) +
                " graus. Deriva horizontal: " + std::to_string(deriva) + " m.");
            return "ROTATED";
        }

        const std::chrono::duration<float> decorrido =
            std::chrono::steady_clock::now() - inicio_;
        if (decorrido.count() > timeout_) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            drone_->log(
                "ERRO: a rotacao nao convergiu em " + std::to_string(timeout_) +
                "s. Faltavam " + std::to_string(erro * 180.0f / static_cast<float>(M_PI)) +
                " graus.");
            return "ERROR";
        }

        drone_->setLocalPosition(
            static_cast<float>(pos_travada_.x()),
            static_cast<float>(pos_travada_.y()),
            static_cast<float>(pos_travada_.z()),
            alvo_);
        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    const float delta_;
    const float graus_;
    Eigen::Vector3d pos_travada_{0.0, 0.0, 0.0};
    float alvo_ = 0.0f;
    float tolerancia_ = 0.0f;
    float timeout_ = 0.0f;
    std::chrono::steady_clock::time_point inicio_;
    bool ok_ = false;
};

#endif  // TESTE_YAW__STATES__ROTATE_STATE_HPP_
