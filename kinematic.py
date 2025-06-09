#----------------------------------------------------------------------------------------------------------------------
#                          TRABALHO FINAL — TÓPICOS ESPECIAIS EM ENGENHARIA DE SISTEMAS III
#                                                    ROBÓTICA MÓVEL
#
#                                                                          Trabalho apresentado à disciplina de Tópicos
#                                                                           Especiais em Engenharia de Sistemas III, do
#                                                                       curso de Bacharelado em Engenharia de Sistemas,
#                                                                            da Universidade Estadual de Montes Claros.
#
#                                                                                    Docente: Dr. Daniel Neri Cardoso
# (c) 20205
# Maria Luiza Xavier; Mateus Moreira Durães
#----------------------------------------------------------------------------------------------------------------------
''' VISÃO GERAL

'''
#-- BIBLIOTECAS -------------------------------------------------------------------------------------------------------
from coppeliasim_zmqremoteapi_client import *
import numpy as np
import signal # Utilizada para controlar a saída da simulação
#-- INICIALIZAÇÃO ----------------------------------------------------------------------------------------------------
client = RemoteAPIClient()
client.timeout = 5
sim = client.require('sim')
#-- FUNÇÕES DE ROTAÇÃO ------------------------------------------------------------------------------------------------
def rotate_x(angle) -> np.ndarray:
    ''' Matriz de Rotação no eixo X '''
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotate_y(angle: float) -> np.ndarray:
    ''' Matriz de Rotação no eixo Y '''
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotate_z(angle: float) -> np.ndarray:
    ''' Matriz de Rotação no eixo Z '''
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
#----------------------------------------------------------------------------------------------------------------------
def cleanup(sig, frame) -> None:
    ''' '''
    print("\033[33mSimulation stopping...\033[0m")

    sim.stopSimulation(True)
    raise SystemExit("\033[33mSimulation stopped.\033[0m")

signal.signal(signal.SIGINT, cleanup)
#-- FUNÇÕES SENSORIAIS ------------------------------------------------------------------------------------------------
def get_position(widget) -> list[float, float, float]:
    return sim.getObjectPosition(widget, -1) # -1 = Referêncial do mundo

def get_orientation(widget) -> list[float, float, float]:
    return sim.getObjectOrientation(widget, -1)

def get_velocity(widget) -> list[float, float]:  # [linear, angular]
    return sim.getVelocity(widget, -1)
#----------------------------------------------------------------------------------------------------------------------
def go_to(robot:tuple, target_position:tuple, target_rotation:float, target_velocity:float) -> np.ndarray:
    ''' Ordena o corpo até um ponto com parâmetros definidos '''

    body, body_reference = robot

    # Ganhos e propriedades
    Ts = 1 # Tempo de assentamento
    zetta = 0.6 # Coeficiente de amortecimento
    w_n = Ts/(4*zetta) # Frequência natural

    Kp = np.array([
        [w_n**2, 0],
        [0, w_n**2]
    ])
    Kd = np.array([
        [2*zetta*w_n, 0],
        [0, 2*zetta*w_n]
    ])

    # Informações sensoriais
    current_position = get_position(body_reference)
    current_rotation = get_orientation(body_reference)
    current_linear_velocity, current_angular_velocity = get_velocity(body)

    # Erros (x~)
    err_position = np.array(current_position[:2]) - np.array(target_position)
    err_orietation = np.array(current_rotation) - np.array(target_rotation)
    err_linear_velocity = np.array(current_linear_velocity) - np.array(target_velocity)

    u = -Kp@err_position -Kd@err_linear_velocity

    return u
#-- FILTRO DE KALMAN --------------------------------------------------------------------------------------------------

#-- VISÃO -------------------------------------------------------------------------------------------------------------

#-- MAIN --------------------------------------------------------------------------------------------------------------
def main() -> None:
    ''' Função principal do sistema '''
    print("\033[32mSimulation started\033[0m")

    # Propriedades do sistema
    mass = 5

    # Componentes
    body = sim.getObject("./Body")
    reference = sim.getObject("./Body/Reference")

    # Loop principal
    sim.startSimulation()
    while True:

        sim.addForceAndTorque(body, [0, 0.1, 0], [0, 0, 0])
        go_to((body, reference),
              target_position=(2, 2),
              target_rotation=(0, 0, 0),
              target_velocity=(0, 0, 0)
            )
#----------------------------------------------------------------------------------------------------------------------
main()