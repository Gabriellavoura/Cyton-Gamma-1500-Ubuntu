# Cyton-Gamma-1500-Ubuntu
Ambiente para utilizar Cyton Gamma 1500 com ROS.

### Dependências
* Ubuntu 18.04 Bionic Beaver;
* Xterm;
> $ sudo apt-get install xterm

* Git;
> $ sudo apt-get install git

* [Gazebo;](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
> $ curl -sSL http://get.gazebosim.org | sh

* [ROS Melodic;](http://wiki.ros.org/melodic/Installation/Ubuntu);

* [ros_control;](http://wiki.ros.org/ros_control)
> $ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers

* [ros_gazebo_control;](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/melodic-devel)
> $ clone from Github, Branch melodic-devel.

* [moveit!](http://moveit.ros.org/install/);
> $ sudo apt-get install ros-melodic-moveit

-----------------------------------------------------------------------------------------------------------------
## Como rodar

Após configurar o ambiente, execute as seguintes ações utilizando o terminal:

1. Crie uma pasta chamada catkin;
Detro desta pasta abra o terminal e insira o seguinte código:
```
$ git clone https://github.com/Gabriellavoura/Cyton-Gamma-1500-Ubuntu.git
```
**Apartir daqui deve ser executado toda vez que for executar o programa**.

Agora digite catkin_make, aguarde compilar e volte para a pasta anterior.

Entre na pasta `/devel/` e digite:
```
$ source setup.bash
```
2. Navegue até a pasta `cyton_gamma_pkg` e execute o arquivo `1500_hardware_launch.sh`:

```
$ cd ~/catkin/Cyton-Gammma-1500-ubuntu/src/cyton_gamma_1500/cyton_gamma_pkg/src/
$ ./1500_hardware_launch.sh
```
3. Abra outro terminal e navegue até a pasta anterior e execute o arquivo `standup.sh`:
```
$ cd ~/catkin/Cyton-Gammma-1500-ubuntu/src/cyton_gamma_1500/
$ ./standup.sh
```
-----------------------------------------------------------------------------------------------------------------
## Para rodar manualmente

Para cada terminal:
```
cd ~/catkin/
source devel/setup.bash
```
- Terminal 1:
```
$ roslaunch cyton_controllers controller_manager.launch
```
- Terminal 2:
```
$ roslaunch cyton_1500_controllers start_controllers.launch
```
Ou
```
$ roslaunch cyton_1500_controllers start_controllers.launch
```
Depois
```
$ rosrun cyton_controllers dynamixel_joint_state_publisher.py
```
- Terminal 3:
```
$ roslaunch cyton_gamma_pkg gamma_1500_planning_execution
```
Ou
```
$ roslaunch cyton_gamma_pkg gamma_300_planning_execution
```
- Terminal 4:
```
$ python /home/rdtintern/catkin_ws/src/cyton_gamma_pkg/src/test_plan.py
```
-----------------------------------------------------------------------------------------------------------------
## Comandos disponíveis para hardware:
Inclui hardware drivers para os motores dynamixel, incluindo um bugfix do "settimeout"; 

Syntax para hardware: 
> roslaunch cyton_1500_controllers controller_manager.launch

> roslaunch cyton_1500_controllers start_controller.launch

> rosrun cyton_1500_controllers dynamixel_joint_state_publisher.py

> roslaunch cyton_gamma_1500_moveit_config moveit_planning_execution.launch 2>/dev/null

Em seguida, um front-end opcional, um front-end combinado ou o comando separado e os feedback.

> rosrun cyton_gamma_pkg combined_front_end.py 2>/dev/null 1500

> rosrun cyton_gamma_pkg command_front_end.py 2>/dev/null 1500

> rosrun cyton_gamma_pkg feedback_front_end.py 2>/dev/null 1500

Ou Use este script de shell, embora atualmente não seja totalmente estável. Se você tentar duas vezes e não funcionar, volte ao lançamento manual.

> ./1500_hardware_launch.sh

## Comandos para Simulação

Syntax para Simulação:

> roslaunch cyton_gamma_pkg simulation_gamma_1500.launch 

Opcionalmente, você pode iniciar os dois aplicativos Qt front-end eles também funcionam na simulação!

Isso iniciará o cliente moveit com o centro de comando RVIZ e a saída de simulação no gazebo.

Foram testados e permitem o controle cartesiano, de espaço conjunto e de força. O controlador de ação da pinça é comandado diretamente, uma vez que os controladores internos do dinamômetro fazem um bom trabalho em uma junta de eixo único.  
