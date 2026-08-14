#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "stdstates/next_waypoints.hpp"
#include "stdstates/arena_point.hpp"

// Estados desta missao. Crie em include/teste_grid_quadrado/states/ e inclua aqui.
#include "teste_grid_quadrado/states/decision_state.hpp"

class TesteGridQuadradoFSM : public fsm::FSM {
public:
    TesteGridQuadradoFSM(
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

        // Le do mapa `params`, e nao da blackboard: `blackboard_get<T>` devolve
        // um PONTEIRO, e o quadrado e uma medida em metros -- arredondar para
        // int transformaria um lado de 1,5 m em 1 m sem avisar.
        this->blackboard_set<std::vector<ArenaPoint>>(
            "waypoints",
            gerar_grid(
                std::get<double>(params.at("lado_quadrado")),
                std::get<double>(params.at("takeoff_height")))
        );

        // ========================= ESTADOS =========================
        this->add_state("ARMING",  std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("GRID", std::make_unique<WaypointListState>());
        this->add_state("DECISION STATE", std::make_unique<DecisionState>());

        // ======================= TRANSICOES ========================
        // Cada linha e: {outcome retornado pelo estado, proximo estado}.
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "GRID"},
            {"ERROR", "ERROR"}
        });

        // Nao declare transicao para "": a FSM trata outcome vazio como "fica
        // no mesmo estado" e nem consulta o mapa. Ja qualquer outro outcome
        // ausente do mapa LANCA std::runtime_error em voo, entao cada lista
        // tem de ser exaustiva.
        this->add_transitions("GRID", {
            {"WAYPOINTS ENDED", "DECISION STATE"},
            {"ERROR", "ERROR"}
        });

        // O laco das voltas. O DecisionState rearma as marcas de `is_visited`
        // antes de devolver "NEXT LAP"; sem isso o GRID sairia de imediato na
        // segunda entrada, com a rota inteira ja marcada como visitada.
        this->add_transitions("DECISION STATE", {
            {"NEXT LAP",  "GRID"},
            {"LAPS DONE", "LANDING"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }

private:
    /// Quadrado de lado `lado` a partir de casa, no plano `altura`.
    ///
    /// Em NED, x e Norte e y e Leste, entao o percurso e: avanca, cruza,
    /// recua, volta. `altura` e negativa (para cima).
    ///
    /// O ULTIMO ponto e a propria casa, e isso importa para repetir a rota: a
    /// volta seguinte comeca de onde a anterior terminou, sem um trecho
    /// diagonal de canto a canto que nao faz parte do quadrado.
    std::vector<ArenaPoint> gerar_grid(double lado, double altura) {
        return {
            {Eigen::Vector3d(lado, 0.0,   altura), false},
            {Eigen::Vector3d(lado, lado,  altura), false},
            {Eigen::Vector3d(0.0,  lado,  altura), false},
            {Eigen::Vector3d(0.0,  0.0,   altura), false}
        };
    }

};

/**
 * @brief No ROS 2 que executa a FSM da missao teste_grid_quadrado.
 *
 * Declara os parametros (sobrescritos pelo YAML no launch), monta a FSM e a
 * executa a 20 Hz.
 */
class TesteGridQuadradoNode : public rclcpp::Node {
public:
    explicit TesteGridQuadradoNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("teste_grid_quadrado_node"), drone_(drone) {

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

            // Parametros do grid
            {"n_voltas", 3.0},
            {"lado_quadrado", 1.0}
        };

        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<TesteGridQuadradoFSM>(drone_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&TesteGridQuadradoNode::executeFSM, this));

        // Trajetoria para o RViz2 (convertida de NED para ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // ACRESCENTE: assinaturas dos nos de visao desta missao. O padrao e o
        // callback escrever na blackboard e os estados apenas lerem de la.
        //
        // cv_sub_ = this->create_subscription<custom_msgs::msg::MinhaDeteccao>(
        //     "minha_deteccao", 10,
        //     std::bind(&TesteGridQuadradoNode::cv_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FSM da missao teste_grid_quadrado iniciada");
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
    std::unique_ptr<TesteGridQuadradoFSM> fsm_;
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
    auto mission_node = std::make_shared<TesteGridQuadradoNode>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
