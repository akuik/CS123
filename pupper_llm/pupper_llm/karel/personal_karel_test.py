import karel
import time

def main():
    pupper = karel.KarelPupper()
    time.sleep(10)
    pupper.move()
    pupper.crab_walk_right()
    pupper.bark()
    
    

if __name__ == '__main__':
    main()
