import pandas as pd

class ResultWriter:
    def __init__(self, filename):
        self.filename = filename
        self.data = {
            'distance': [],
            'accuracy': []
            }

    def write_row(self, distance, accuracy):
        self.data['distance'].append(distance)
        self.data['accuracy'].append(accuracy)

    def write_to_csv(self):
        df = pd.DataFrame(self.data)
        df.to_csv(self.filename, index=False)