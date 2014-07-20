import sys
import os.path
import csv
from collections import defaultdict

class keydefaultdict(defaultdict):
    '''Like defaultdict, but the default_factory function is called with the
       requested key as an argument'''
    def __missing__(self, key):
        if self.default_factory is None:
            raise KeyError(key)
        else:
            self[key] = ret = self.default_factory(key)
            return ret

input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]

reader = csv.DictReader(open(input_path, 'rb'))

def make_csv_writer(gridname):
    'Uses input_filename and reader.fieldnames globals'
    path = "%s_%s.csv" % (input_filename, gridname)
    writer = csv.DictWriter(open(path, 'wb'), reader.fieldnames)
    writer.writeheader()
    return writer
    
gridname_to_csv_writer = keydefaultdict(make_csv_writer)

for row in reader:
    gridname_to_csv_writer[row['Grid Name']].writerow(row)
