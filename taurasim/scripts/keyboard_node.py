import pygame
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

#!/usr/bin/env python3
# coding=utf-8
"""
    File:
        keyboard_node.py

    Description:
        Simple python routine to watch the keyboard or a joystick
        to send velocity commands to a Gazebo simulation.
"""



# Vamos acompanhar o estado dessas teclas
KEYS = [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d]

# Indice dos eixos x e y do joystick
X_AXIS = 0
Y_AXIS = 4
INVERT_X_AXIS = True
INVERT_Y_AXIS = True

ROBOTS = 3

# Namespace dos tópicos que iremos publicar
DEFAULT_NAMESPACE = "/yellow_team/robot_"

DEFAULT_DEBUG = False

# A vel máxima do robô é 2 m/s
MAX_LIN_VEL = 1.0  # m/s

# A vel máxima do robô é 40 rad/s
MAX_ROT_VEL = 10  # rad/s

# Define a rampa de aceleração quando usando o teclado
# Valores em porcentagem da velocidade máxima
KEYBOARD_LINEAR_STEP = 0.03
KEYBOARD_LINEAR_MAX = 1.0

KEYBOARD_ANGULAR_STEP = 0.03
KEYBOARD_ANGULAR_MAX = 0.6

# Os comandos vão de -126 até 126 de modo que os bytes 0xFE e 0xFF
# nunca são utilizados
SCALE = 126


def getNamespace(number):
    return DEFAULT_NAMESPACE + f'{number}'


def drawConsole(win, font, console):
    """
    Fills window console with the sentences stored in the list console
        :param win: pygame.display Window object to be filled
        :param font: pygame.Font Font style to be used
        :param console: list<font.render> List of text to write
    """
    img = font.render("Event console Area", 1, (155, 155, 155), (0, 0, 0))
    win.blit(img, (2, 132))
    ypos = 450
    h = list(console)
    h.reverse()
    for line in h:
        r = win.blit(line, (10, ypos))
        win.fill(0, (r.right, r.top, 620, r.height))
        ypos -= font.get_height()


class KeyboardNode(Node):
    def __init__(self, debug=DEFAULT_DEBUG):
        super().__init__('vss_human_controller')

        self.vel_pub = []
        self.current_robot = 0

        for i in range(ROBOTS):
            self.vel_pub.append(self.create_publisher(
                Twist, getNamespace(i) + '/cmd_vel', 2))

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

        pygame.init()

        # Cria a janela
        self.win = pygame.display.set_mode((640, 480), pygame.RESIZABLE)
        pygame.display.set_caption("Keyboard Comunication Interface")

        # Lista de frases a serem mostradas no console
        self.console = []
        self.font = pygame.font.Font(None, 26)

        # Dicionário para guardar o estado de algumas teclas
        self.state = {}
        for key in KEYS:
            self.state[key] = False

        # Vamos detectar os Joysticks conectados ao computador
        self.axis = [0.0, 0.0]
        self.using_joystick = True
        for x in range(pygame.joystick.get_count()):
            j = pygame.joystick.Joystick(x)
            j.init()
            txt = "Enabled joystick: " + j.get_name()
            print(txt)
            img = self.font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            self.console.append(img)

        if not pygame.joystick.get_count():
            self.using_joystick = False
            print("No Joysticks to Initialize")
            img = self.font.render("No Joysticks to Initialize", 1,
                                   (50, 200, 50), (0, 0, 0))
            self.console.append(img)

        self.vel_lin = 0.0
        self.vel_ang = 0.0

        self.running = True

    def timer_callback(self):
        for e in pygame.event.get():

            # Movimento dos botões do teclado
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    self.running = False
                if e.key in KEYS:
                    self.state[e.key] = True
            elif e.type == pygame.KEYUP:
                if e.key in KEYS:
                    self.state[e.key] = False

            # Movimento dos direcionais do joystick
            if e.type == pygame.JOYAXISMOTION:
                if e.dict['axis'] in (X_AXIS, Y_AXIS):
                    if e.dict['axis'] == X_AXIS:
                        if INVERT_X_AXIS:
                            self.axis[0] = -e.value
                        else:
                            self.axis[0] = e.value

                    elif e.dict['axis'] == Y_AXIS:
                        if INVERT_Y_AXIS:
                            self.axis[1] = -e.value
                        else:
                            self.axis[1] = e.value

            # Caso algum botão do joystick seja apertado
            if e.type == pygame.JOYBUTTONDOWN \
                or e.type == pygame.JOYBUTTONUP \
                    or e.type == pygame.JOYHATMOTION:

                txt = "%s: %s" % (pygame.event.event_name(e.type), e.dict)
                print(txt)
                img = self.font.render(txt, 1, (50, 200, 50), (0, 0, 0))
                self.console.append(img)
                self.console = self.console[-13:]

            # L1 pressionado
            if (e.type == pygame.JOYBUTTONDOWN and e.dict['button'] == 4) or (e.type == pygame.KEYDOWN and e.key == pygame.K_e):
                self.current_robot += 1
                self.current_robot %= ROBOTS

            # R1 pressionado
            if (e.type == pygame.JOYBUTTONDOWN and e.dict['button'] == 5) or (e.type == pygame.KEYDOWN and e.key == pygame.K_q):
                self.current_robot -= 1
                self.current_robot %= ROBOTS

            elif e.type == pygame.VIDEORESIZE:
                self.win = pygame.display.set_mode(e.size, pygame.RESIZABLE)

            elif e.type == pygame.QUIT:
                self.running = False

        drawConsole(self.win, self.font, self.console)
        pygame.display.flip()

        if self.using_joystick:
            txt = f"Linear: {int(self.axis[1]*SCALE)} Angular: {int(self.axis[0]*SCALE)}"
            img = self.font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            self.console.append(img)
            self.console = self.console[-13:]

            vel_cmd_twist = Twist()
            vel_cmd_twist.linear.x = self.axis[1]*MAX_LIN_VEL
            vel_cmd_twist.angular.z = self.axis[0]*MAX_ROT_VEL

            self.vel_pub[self.current_robot].publish(vel_cmd_twist)

        else:
            if self.state[pygame.K_w] and not self.state[pygame.K_s]:
                self.vel_lin += KEYBOARD_LINEAR_STEP
                self.vel_lin = min(self.vel_lin, KEYBOARD_LINEAR_MAX)
            elif self.state[pygame.K_s] and not self.state[pygame.K_w]:
                self.vel_lin -= KEYBOARD_LINEAR_STEP
                self.vel_lin = max(self.vel_lin, -KEYBOARD_LINEAR_MAX)
            else:
                self.vel_lin = 0.0

            if self.state[pygame.K_a] and not self.state[pygame.K_d]:
                self.vel_ang += KEYBOARD_ANGULAR_STEP
                self.vel_ang = min(self.vel_ang, KEYBOARD_ANGULAR_MAX)
            elif self.state[pygame.K_d] and not self.state[pygame.K_a]:
                self.vel_ang -= KEYBOARD_ANGULAR_STEP
                self.vel_ang = max(self.vel_ang, -KEYBOARD_ANGULAR_MAX)
            else:
                self.vel_ang = 0.0

            txt = f"Linear: {int(self.vel_lin*SCALE)} Angular: {int(self.vel_ang*SCALE)}"
            img = self.font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            self.console.append(img)
            self.console = self.console[-13:]

            vel_cmd_twist = Twist()
            vel_cmd_twist.linear.x = self.vel_lin * MAX_LIN_VEL
            vel_cmd_twist.angular.z = self.vel_ang * MAX_ROT_VEL

            self.vel_pub[self.current_robot].publish(vel_cmd_twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()