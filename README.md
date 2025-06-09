<!-- ===================================================================== -->
<!--                       README – Navegação Autônoma ROS1 🤖               -->
<!-- ===================================================================== -->
# Sistema de Navegação Autônoma com ROS1 Noetic 🤖

Projeto para exploração de ambientes mapeados, planejamento de trajetórias (Dijkstra e RRT*) e navegação autônoma usando **ROS Noetic** em **Ubuntu 20.04 LTS**.

<p align="center">
  <img src="Images/navigation_demo.png" alt="Demonstração da navegação autônoma">
</p>

<div align="center">
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)]  
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)]  
[![Build Status](https://img.shields.io/badge/build-catkin--make-brightgreen)]  
[![License](https://img.shields.io/github/license/usuario/repositorio.svg)](LICENSE)
</div>

---

<div align="center">
• [Estrutura do Repositório](#estrutura-do-repositório-📂)  
• [Visão Geral](#visão-geral-🗺️)  
• [Pacotes ROS Utilizados](#pacotes-ros-utilizados)  
• [Arquitetura & Algoritmos](#arquitetura--algoritmos-⚙️)  
• [Requisitos Atendidos](#requisitos-atendidos-✅)  
• [Como Compilar e Rodar](#como-compilar-e-rodar-🚀)  
• [Contribuição](#contribuição-🤝)  
• [Licença](#licença-📄)  
• [Membros](#membros-👥)  
</div>

---

## Estrutura do repositório 📂

```bash
turtlebot3_2dnav/
├── config/                           
│   ├── costmap_common_params.yaml    # Parâmetros comuns do costmap
│   ├── global_costmap_params.yaml    # Config. do costmap global
│   ├── local_costmap_params.yaml     # Config. do costmap local
│   └── rrt_global_planner_params.yaml# Parâmetros do planner RRT*
├── include/                          
│   ├── rrt_planner.h                 
│   └── vertex.h                      
├── launch/                           
│   ├── mapping.launch                # Lança SLAM e RViz
│   ├── mecanum.launch                # Inicializa robô mecanum
│   ├── move_base.launch              # Configuração do move_base
│   └── my_robot_configuration.launch # Carrega URDF
├── maps/                             
│   ├── main_map.pgm                  
│   ├── main_map.yaml                 
│   └── willow_garage_map.*           
├── msg/                              
│   └── PlanningMetrics.msg           # Mensagem customizada
├── scripts/                          
│   ├── odometry.cpp                  # Publica odometria
│   ├── publish_PointClouds.cpp       # Publica nuvem de pontos
│   └── publishing_GoalPoses.py       # Publica waypoints no move_base
├── src/                              
│   ├── rrt_planner.cpp               # Plugin RRT* customizado
│   └── vertex.cpp                    # Estrutura de vértices para RRT*
├── urdf/                             
│   ├── summit_base.urdf              # Robô base
│   └── mecanum.xacro                 # Robô com 4 rodas mecanum e LiDAR
├── CMakeLists.txt                    
└── package.xml                        
```

## Visão geral 🗺️

| Componente                             | Tipo               | Responsabilidade Principal                           | Tópicos / Ferramentas ROS          |
|----------------------------------------|--------------------|------------------------------------------------------|------------------------------------|
| **`scripts/publishing_GoalPoses.py`**  | Nó Python          | Publica uma sequência de GoalPoints no `move_base`   | `rospy`, `actionlib`, `move_base_msgs` |
| **`src/rrt_planner.cpp`**              | Plugin C++         | Planejador Global RRT* customizado                   | `nav_core`, `pluginlib`            |
| **`launch/move_base.launch`**          | Launch file        | Inicializa `move_base` com AMCL e planners configurados | `roslaunch`, `amcl`, `nav_core`    |
| **`urdf/mecanum.xacro`**               | Modelo URDF/Xacro  | Robô holonômico com 4 rodas mecanum e sensor LiDAR   | `robot_state_publisher`, `gazebo_ros` |
| **`config/rrt_global_planner_params.yaml`** | Configuração       | Parâmetros do planner RRT* (iterações, step size)    | `rosparam`                         |

## Pacotes ROS Utilizados

| Categoria                    | Pacote / Ferramenta               | Função na Solução                                        |
|------------------------------|-----------------------------------|----------------------------------------------------------|
| **Localização & Mapeamento** | `amcl`, `gmapping`                | Localização probabilística e SLAM em 2D                  |
| **Navegação**                | `move_base`, `nav_core`           | Planejamento global (Dijkstra, RRT*) e controle local    |
| **Simulação**                | `gazebo_ros`, `turtlebot3_gazebo` | Simulação do robô mecanum em ambientes pré-mapeados      |
| **Comunicação**              | `rospy`, `actionlib`, `pluginlib` | Execução de nós Python e C++, e carregamento de plugins  |
| **Visualização**             | `rviz`                            | Visualização de mapas, trajetórias e estados             |

## Arquitetura & Algoritmos ⚙️

### 1. SLAM & Mapeamento
- Uso de `gmapping` para gerar mapa 2D (occupancy grid) em tempo real.  
- Publicação de `/map`, `/odom` e frames TF.

### 2. Planejamento de Trajetória
#### Dijkstra (nativo do move_base)
- Busca de caminho em grid de ocupação.  
- Planner global padrão.

#### RRT* (Plugin customizado)
1. Amostragem uniforme no espaço livre.  
2. Conexão incremental de vértices.  
3. Re-wiring para otimização de custo.  
4. Publicação da árvore e do caminho em `/plan`.

### 3. Movimentação Holonômica
- 4 rodas mecanum simuladas pelo plugin **PlanarMove**.  
- Comandos de velocidade publicados em `/cmd_vel`.

## Requisitos Atendidos ✅
- **Navegação autônoma** em ambiente mapeado, evitando obstáculos.  
- **URDF** do robô com LiDAR e mobilidade mecanum.  
- **Configuração** de `amcl`, `move_base` e `nav_core`.  
- **Implementação de plugin** RRT* para planejamento de rota.  
- **Script** para navegação por waypoints pré-definidos.  
- **Visualização** de trajetórias planejadas no RViz.

## Como compilar e rodar 🚀

```bash
# 1. Clone o repositório
cd ~/catkin_ws/src
git clone https://github.com/usuario/turtlebot3_2dnav.git

# 2. Instalar dependências automaticamente
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar o workspace
catkin_make
source devel/setup.bash

# 4. Executar a simulação e navegação

# Terminal A: SLAM e RViz
roslaunch turtlebot3_2dnav mapping.launch

# Terminal B: move_base com AMCL e planners
roslaunch turtlebot3_2dnav move_base.launch

# Terminal C: Publica waypoints para navegação
rosrun turtlebot3_2dnav publishing_GoalPoses.py
```

## Contribuição 🤝

Contribuições são bem-vindas!  
Abra uma **Issue** para discutir melhorias ou enviar sugestões, ou submeta um **Pull Request** para correções, novos recursos ou ajustes de documentação.

## Licença 📄

Este projeto está licenciado sob a **MIT License**.  
Consulte o arquivo [LICENSE](LICENSE) para mais detalhes.

## Membros 👥

| Nome                  | GitHub                                      |
|-----------------------|---------------------------------------------|
| Seu Nome              | [@seu_usuario](https://github.com/seu_usuario) |
| Outro Membro          | [@outro_usuario](https://github.com/outro_usuario) |

