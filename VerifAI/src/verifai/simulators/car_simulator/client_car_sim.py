from verifai.client import Client

class ClientCar(Client):

    def __init__(self, simulation_data):
        port = simulation_data.port
        bufsize = simulation_data.bufsize
        super().__init__(port, bufsize)
        self.simulation = simulation_data.simulation

    def simulate(self, sample):
        return self.simulation(sample)



