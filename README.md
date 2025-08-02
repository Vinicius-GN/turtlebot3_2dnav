<!-- ===================================================================== -->
<!--                       README – Navegação Autônoma ROS1 🤖               -->
<!-- ===================================================================== -->
# Sistema de Navegação Autônoma com ROS1 Noetic 🤖

Projeto para exploração de ambientes mapeados, planejamento de trajetórias (Dijkstra e RRT*) e navegação autônoma usando **ROS Noetic** em **Ubuntu 20.04 LTS**.

<p align="center">
  [Link para o vídeo de apresentação](
</p>

<div align="center">
  
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)
![Build Status](https://img.shields.io/badge/build-catkin--make-brightgreen)

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
│   └── rrt_global_planner_params.yaml # Parâmetros do planner RRT*
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
│   └── publishing_GoalPoses.py       # Publica waypoints no move_base
├── src/                              
│   ├── rrt_planner.cpp               # Plugin RRT* customizado
│   └── vertex.cpp                    # Estrutura de vértices para RRT*
├── urdf/                             
│   ├── summit_base.urdf              # Robô com 4 rodas mecanum e LiDAR
├── CMakeLists.txt                    
└── package.xml                        
```

## Visão geral 🗺️

| Componente                             | Tipo               | Responsabilidade Principal                           | Tópicos / Ferramentas ROS          |
|----------------------------------------|--------------------|------------------------------------------------------|------------------------------------|
| **`scripts/publishing_GoalPoses.py`**  | Nó Python          | Publica uma sequência de GoalPoints no `move_base`   | `rospy`, `actionlib`, `move_base_msgs` |
| **`src/rrt_planner.cpp`**              | Plugin C++         | Planejador Global RRT* customizado                   | `nav_core`, `pluginlib`            |
| **`launch/move_base.launch`**          | Launch file        | Inicializa `move_base` com AMCL e planners configurados | `roslaunch`, `amcl`, `nav_core`    |
| **`urdf/summit_base.urdf`**               | Modelo URDF/Xacro  | Robô holonômico com 4 rodas mecanum e sensor LiDAR   | `robot_state_publisher`, `gazebo_ros` `plannar_move` |
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

#### D\* Lite (Plugin customizado)
1. Inicialização dos valores de custo e heurística no grid.
2. Cálculo do caminho ótimo do ponto atual até o destino (similar ao A\*).
3. Monitoramento do ambiente para detectar mudanças (ex.: novos obstáculos).
4. Atualização incremental do caminho a partir das mudanças percebidas.
5. Repriorização dos nós afetados na fila de prioridade.
6. Geração de nova rota otimizada com mínimo custo até o destino.
7. Publicação contínua do caminho atualizado em `/plan`.


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
git clone https://github.com/Vinicius-GN/turtlebot3_2dnav/

# 2. Instalar dependências automaticamente
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar o workspace
catkin_make
source devel/setup.bash

# 4. Executar a simulação e navegação

# Terminal A: move_base com AMCL e planners com mapa estático gerado por SLAM
roslaunch turtlebot3_2dnav mecanum.launch

# Terminal B: Publica waypoints para navegação
rosrun turtlebot3_2dnav publishing_GoalPoses.py
```

**Comportamento Esperado:**
Navegação pelo mapa baseando-se em pontos pré-definidos deve se iniciar e finalizar de forma autônoma com o algoritmo escolhido na linha do arquivo "mecanum.launch":
```bash
##A linha descomentada seleciona o algoritmo de planejamento global utilizado na navegação.
<!-- <arg name="base_global_planner" default="navfn/NavfnROS"/> -->
<arg name="base_global_planner" default="rrt_planning/RRTPlanner"/>
<!-- <arg name="base_global_planner" default="srl_dstar_lite/SrlDstarLite"/> --> //Esse está em processo de otimização.
```

## Contribuição 🤝

Contribuições são bem-vindas!  
Abra uma **Issue** para discutir melhorias ou enviar sugestões, ou submeta um **Pull Request** para correções, novos recursos ou ajustes de documentação.

## Licença 📄

Este projeto está licenciado sob a **MIT License**.  
Consulte o arquivo [LICENSE](LICENSE) para mais detalhes.

## Membros 👥

| Nome                  | GitHub                                      | Número USP                                     |
|-----------------------|---------------------------------------------|---------------------------------------------|
| Vinicius             | [@Vinicius-GN](https://github.com/Vinicius-GN) | 14749363                              |

