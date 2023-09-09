import random
import move_tasks


def main():
    x, y = random.randint(-10, 10), random.randint(-10, 10)
    move_tasks.MoveToPoseGlobalTask(x, y, 0, 0, 0, 0, None, None).execute(None)
