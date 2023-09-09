import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((200, 200))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

if __name__ == '__main__':
    init()
