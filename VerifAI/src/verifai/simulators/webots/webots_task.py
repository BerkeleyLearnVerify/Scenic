class webots_task():
    def __init__(self, N_SIM_STEPS, supervisor):
        self.N_SIM_STEPS = N_SIM_STEPS
        self.supervisor = supervisor

    def use_sample(self, sample):
        raise NotImplementedError("Define in child class")

    def run_task(self, sample):
        raise NotImplementedError("Define in child class")

    def close(self):
        self.supervisor.simulationReset()
