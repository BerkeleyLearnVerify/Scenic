from abc import ABC, abstractmethod
import socket
import dill


class Client(ABC):
    """Generic client for running simulations based on samples from the server.

    Users must implement the abstract method `simulate` to run a simulation.
    """

    def __init__(self, port, bufsize):
        self.port = port
        self.bufsize = bufsize

    def initialize(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = '127.0.0.1'
        try:
            self.socket.connect((self.host, self.port))
        except ConnectionResetError:
            self.close()
            return False
        except OSError as e:
            self.close()
            raise RuntimeError('unable to connect to server') from e
        return True

    def receive(self):
        data = []
        try:
            while True:
                msg = self.socket.recv(self.bufsize)
                if not msg:
                    break
                data.append(msg)
        except OSError:
            return False, None
        data = dill.loads(b"".join(data))
        return True, data

    def send(self, data):
        msg = dill.dumps(data)
        self.socket.send(msg)
        self.socket.shutdown(socket.SHUT_WR)

    def close(self):
        self.socket.close()

    def run_client(self):
        success = self.initialize()
        try:
            if success:
                success, sample = self.receive()
            if not success:
                print("No new sample received from server.")
                return False
            sim = self.simulate(sample)
            self.send(sim)
            return True
        finally:
            self.close()

    @abstractmethod
    def simulate(self, sample):
        """Run a simulation from the given sample.

        Returns:
            The outcome of the simulation (e.g. trajectories of objects), to be
            passed to the monitor.
        """
        pass
