from DecisionMaking.BehaviorTree.Task import Action

class Odometer(Action):
    def init(self):
        self.deltax = 0
        self.deltay = 0

    def tick(self):
        self.walk(0, 1, 0)
        self.deltax += self.bb.delta.x
        self.deltay += self.bb.delta.y

        print 'delta', self.bb.delta, 'dx', self.deltax, 'dy', self.deltay
