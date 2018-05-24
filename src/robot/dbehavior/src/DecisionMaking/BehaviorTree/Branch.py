from Task import Task, Action, Status

def selector(*args):
    class Selector(Task):
        """
        Runs children in sequence until one succeeds or running,
        if all failes, return Failure
        """

        def __init__(self):
            super(Selector, self).__init__()
            self.currentIndex = 0
            for child in args:
                self.addChild(child)

        def tick(self):
            """
            Tick until some child returns not failure, if status is running,
            next time we start here.
            """
            if len(self._children) is 0:
                raise TypeError('Selector empty')

            for index in range(self.currentIndex, len(self._children)):
                child = self._children[index]
                # if child.isFresh():
                #     child.onEnter()

                status = child.tick()
                # print 'selector', status, child.__class__

                if status is None:
                    raise Exception('Node tick not return status')

                if status is Status.RUNNING:
                    self.currentIndex = index
                    return self.running()
                elif status is Status.SUCCESS:
                    self.currentIndex = 0
                    self.fresh()
                    return self.success()

            # We runs out of children and no one returns Success
            # So we returns failure

            self.currentIndex = 0
            self.fresh()
            return self.failure()

    return Selector


def sequence(*args):
    class Sequence(Task):
        def __init__(self):
            super(Sequence, self).__init__()
            self.currentIndex = 0
            for child in args:
                self.addChild(child)

        def tick(self):
            if len(self._children) is 0:
                raise TypeError('Sequence empty!')

            for index in range(self.currentIndex, len(self._children)):
                child = self._children[index]
                if child.isFresh():
                    child.onEnter()

                status = child.tick()

                if status is None:
                    raise Exception('Node tick not returning status')

                if status is Status.RUNNING:
                    self.currentIndex = index
                    return self.running()
                elif status is Status.FAILURE:
                    self.currentIndex = 0
                    self.fresh()
                    return self.failure()

            # Here we runs out of children, and no one returns failure
            # So we return success
            self.fresh()
            self.currentIndex = 0
            return self.success()

    return Sequence

def parallel(*args):
    class Parallel(Task):
        """
        Executes all children simultaneously. Fails when any of them returns failure.
        """

        def __init__(self):
            super(Parallel, self).__init__()
            for child in args:
                self.addChild(child)

        def init(self):
            self.numSuccess = 0


        def tick(self):
            for child in self._children:
                if child.isFresh():
                    child.onEnter()

                status = child.tick()

                if status is None:
                    raise Exception('Node tick not returning status')

                if status is Status.SUCCESS:
                    self.numSuccess += 1
                elif status is Status.FAILURE:
                    self.fresh()
                    return self.failure()


            if self.numSuccess is len(self._children):
                self.fresh()
                return self.success()
            else:
                self.numSuccess = 0
                return self.running()

    return Parallel
