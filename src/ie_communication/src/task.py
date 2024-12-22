

class Task:
    def __init__(self, name):
        self._params = None
        self._running = False
        self._name = name
        
    @property
    def params(self):
        return self._params
    
    @property
    def running(self):
        return self._running
    
    @params.setter
    def params(self, params):
        self._params = params
    
