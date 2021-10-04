import multiprocessing


process_list = []
process_id = 1


class ProcessWrapper:

    def __init__(self):
        global process_id

        super().__init__()

        self.daemon = True
        self.process = None

        process_list.append([process_id, self])

        process_id += 1

    def run(self, func):
        self.process = multiprocessing.Process(target=exec, args=func)
        self.process.start()

    def kill(self):
        self.process.terminate()

        # process_list.remove([process_id, self])
