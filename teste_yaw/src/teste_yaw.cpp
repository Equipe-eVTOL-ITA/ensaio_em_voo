#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"

// Estados desta missao.
#include "teste_yaw/states/rotate_state.hpp"
#include "teste_yaw/states/advance_state.hpp"
#include "teste_yaw/states/decision_state.hpp"

/**
 * @brief Maquina de estados da missao teste_yaw.
 *
 * Ensaio de ROTACAO, irmao do teste_grid_quadrado. Aquele exercita a
 * translacao; este exercita o cabeceio, que e a parte que costuma surpreender
 * -- o yaw vive em (-pi, pi] e a descontinuidade em pi ja custou um bug de
 * competicao inteiro (veja o YawSweepState do stdstates).
 *
 * A manobra, repetida n_voltas vezes:
 *
 *     gira 90 graus  ->  avanca r metros  ->  gira 180 graus  ->  avanca r
 *
 * As duas pernas de avanco sao no rumo em que o drone esta apontando, e nao
 * para uma coordenada fixa: e isso que faz o teste medir o cabeceio, e nao a
 * navegacao. Como a segunda perna e no rumo oposto da primeira, o drone volta
 * ao ponto de partida a cada volta -- e o quanto ele NAO volta e o erro
 * acumulado que o DecisionState registra.
 */
class TesteYawFSM : public fsm::FSM {
public:
    TesteYawFSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // O drone fica na blackboard: todo estado o acessa por aqui.
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // Parametros do ROS 2 (vindos do YAML) viram entradas da blackboard.
        for (const auto &[key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // ========================= ESTADOS =========================
        this->add_state("ARMING",  std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // Quatro pernas, quatro estados. Duas instancias da mesma classe em
        // cada par: o que muda entre "ROTATE 90" e "ROTATE 180" e so o
        // argumento, e entre os dois avancos, nada -- mas cada estado precisa
        // de um nome proprio, porque e o nome que a FSM usa para transitar.
        //
        // Poderia ser um estado so, com a manobra inteira dentro. Seria pior:
        // o nome do estado atual e o que aparece no log a 20 Hz, e num ENSAIO
        // saber em qual perna o drone esta quando algo sai errado e metade do
        // valor da ferramenta.
        this->add_state("ROTATE 90",  std::make_unique<RotateState>(90.0f));
        this->add_state("ADVANCE 1",  std::make_unique<AdvanceState>());
        this->add_state("ROTATE 180", std::make_unique<RotateState>(180.0f));
        this->add_state("ADVANCE 2",  std::make_unique<AdvanceState>());
        this->add_state("DECISION",   std::make_unique<DecisionState>());

        // ======================= TRANSICOES ========================
        // Cada linha e: {outcome retornado pelo estado, proximo estado}.
        //
        //   ARMING -> TAKEOFF -> DECISION -> ROTATE 90 -> ADVANCE 1
        //                            ^                          |
        //                            |                          v
        //                        ADVANCE 2 <-------------- ROTATE 180
        //                            | (n_voltas completas)
        //                            v
        //                         LANDING -> FINISHED
        //
        // O DECISION vem ANTES da primeira volta, e nao depois: e ele que grava
        // a origem contra a qual o erro de cada volta e medido, e essa leitura
        // so vale depois do TAKEOFF -- que reancora o referencial do mundo.
        // Veja o comentario no decision_state.hpp.
        //
        // Nao declare transicao para "": a FSM trata outcome vazio como "fica
        // no mesmo estado" e nem consulta o mapa. Ja qualquer outro outcome
        // ausente do mapa LANCA std::runtime_error em voo, entao cada lista
        // tem de ser exaustiva.
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "DECISION"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("ROTATE 90", {
            {"ROTATED", "ADVANCE 1"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("ADVANCE 1", {
            {"ADVANCED", "ROTATE 180"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("ROTATE 180", {
            {"ROTATED", "ADVANCE 2"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("ADVANCE 2", {
            {"ADVANCED", "DECISION"},
            {"ERROR", "ERROR"}
        });

        // O laco das voltas. Cada estado da manobra recalcula o proprio alvo no
        // on_enter, entao reentrar em "ROTATE 90" ja parte do cabeceio novo --
        // nao ha nada a rearmar, diferente do teste_grid_quadrado.
        this->add_transitions("DECISION", {
            {"NEXT LAP",  "ROTATE 90"},
            {"LAPS DONE", "LANDING"},
            {"ERROR", "ERROR"}
        });

        // Nenhum estado desta missao devolve "" para sempre: as duas pernas de
        // rotacao e as duas de avanco tem teto de tempo e caem em "ERROR" se
        // nao convergirem. Um ensaio que trava pairando nao diz o que falhou.

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

/**
 * @brief No ROS 2 que executa a FSM da missao teste_yaw.
 *
 * Declara os parametros (sobrescritos pelo YAML no launch), monta a FSM e a
 * executa a 20 Hz.
 */
class TesteYawNode : public rclcpp::Node {
public:
    explicit TesteYawNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("teste_yaw_node"), drone_(drone) {

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e trocar
        // de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Decolagem (lidos pelo TakeoffState)
            {"takeoff_height",          -2.5},   // metros, NED: negativo e para cima
            {"max_vertical_velocity",    1.2},
            {"position_tolerance",       0.15},

            // Pouso (lidos pelo LandingState)
            {"landing_velocity_max",     0.5},
            {"landing_velocity_min",     0.15},
            {"max_base_height",          0.5},
            {"landing_timeout",          5.0},

            // Movimento horizontal
            {"max_horizontal_velocity",  1.5},

            // Parametros do ensaio
            {"n_voltas",                 3.0},
            {"distancia_avanco",         2.0},   // o "r" da manobra, em metros

            // Tolerancia de cabeceio, em radianos. 0.05 rad ~ 2.9 graus, o
            // mesmo valor que o move_local_constant_step do drone_lib usa como
            // "yaw_ok" -- apertar mais aqui do que a biblioteca aceita la faria
            // este ensaio reprovar manobras que as missoes dao por boas.
            {"yaw_tolerance",            0.05},

            // Tetos de tempo por perna. Sem eles, uma perna que nao converge
            // deixa o drone pairando indefinidamente, sem erro no log.
            {"rotation_timeout",        15.0},
            {"advance_timeout",         30.0},
        };

        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<TesteYawFSM>(drone_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&TesteYawNode::executeFSM, this));

        // Trajetoria para o RViz2 (convertida de NED para ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // ACRESCENTE: assinaturas dos nos de visao desta missao. O padrao e o
        // callback escrever na blackboard e os estados apenas lerem de la.
        //
        // cv_sub_ = this->create_subscription<custom_msgs::msg::MinhaDeteccao>(
        //     "minha_deteccao", 10,
        //     std::bind(&TesteYawNode::cv_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FSM da missao teste_yaw iniciada");
    }

private:
    void executeFSM() {
        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        // NED -> ENU para visualizar no RViz2.
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp       = this->now();
        ps.header.frame_id    = "map";
        ps.pose.position.x    =  static_cast<float>(pos.y());   // East  = NED y
        ps.pose.position.y    =  static_cast<float>(pos.x());   // North = NED x
        ps.pose.position.z    = -static_cast<float>(pos.z());   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        // Log de estado e posicao a cada 2 s (40 ticks a 20 Hz).
        if (log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] pos=(%.2f, %.2f, %.2f) yaw=%.2f rad",
                        fsm_->get_current_state().c_str(),
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        static_cast<float>(pos.z()),
                        static_cast<float>(orient[2]));
        }

        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "FSM terminou com: %s",
                        fsm_->get_fsm_outcome().c_str());
            rclcpp::shutdown();
        }
    }

    /// Declara cada parametro com seu padrao e le o valor efetivo.
    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>> &defaults) {

        std::map<std::string, std::variant<double, std::string>> result;
        for (const auto &[name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }
        return result;
    }

    std::shared_ptr<Drone> drone_;
    std::unique_ptr<TesteYawFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
    int log_counter_ = 0;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // O Drone JA sobe o proprio executor e a propria thread de spin no
    // construtor (veja drone_lib/src/Drone.cpp). Adiciona-lo a um executor
    // aqui lanca em tempo de execucao:
    //
    //     terminate called after throwing an instance of 'std::runtime_error'
    //       what():  Node '/Drone' has already been added to an executor.
    //
    // Por isso so o no da missao entra no executor deste main.
    auto drone        = std::make_shared<Drone>();
    auto mission_node = std::make_shared<TesteYawNode>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
