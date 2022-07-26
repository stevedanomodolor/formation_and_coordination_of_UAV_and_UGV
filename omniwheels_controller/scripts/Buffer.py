import numpy as np 


class Buffer:
    def __init__(self, buffer_size):
        self.buf = np.zeros(buffer_size)
        self.bfs = buffer_size
        
    def get_mean(self):
        return self.buf.mean()
    def fill(self, data):
        self.buf[0:self.bfs-1] = self.buf[1:self.bfs]
        self.buf[self.bfs-1] = data
    def print_buffer(self):
        print(self.buf)
