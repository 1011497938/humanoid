from DecisionMaking.BehaviorTree.Task import Action
from utils.VecPos import VecPos
from Timer import Timer

class ScanField_fast(Action):
    def init(self):
        self.path = [VecPos(15, -90),
                     # VecPos(20, -50),
                     VecPos(15, 0),
                     # VecPos(20, 50),
                     VecPos(15, 90),
                     # VecPos(40, 0)
                     ]
        self.keep = False
        self.timer = Timer(0.75)
        self.iter = iter(self.path)
        self.curplat = self.iter.next()

    def tick(self):
        """
        Success when see both goal or see circle or time out
        """
        cur = self.curplat
        self.lookAt(cur.x, cur.y, 10, 10)

        if self.headGotDest(cur.x, cur.y):
            if not self.keep:
                self.keep = True
                self.timer.restart()
            elif self.timer.finished():
                self.keep = False
                try:
                    self.curplat = self.iter.next()
                except StopIteration:
                    self.path.reverse()
                    self.iter = iter(self.path)
                    self.iter.next()
                    self.curplat = self.iter.next()

        return self.success()


scanField_fast = ScanField_fast()
