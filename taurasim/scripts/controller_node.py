import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Nenhum joystick detectado")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detectado: {joystick.get_name()}")

    while True:
        for event in pygame.event.get():
            print(event)
